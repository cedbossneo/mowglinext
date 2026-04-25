package foxglove

import (
	"encoding/binary"
	"encoding/json"
	"fmt"
	"math"
)

// SerializeCDR serializes a JSON-encoded ROS2 message into CDR binary format
// using the given schema. Returns bytes including the 4-byte encapsulation header.
func SerializeCDR(jsonData []byte, schema *msgSchema) ([]byte, error) {
	var msg map[string]interface{}
	if err := json.Unmarshal(jsonData, &msg); err != nil {
		return nil, fmt.Errorf("cdr write: unmarshal: %w", err)
	}

	// ROS 2 Kilted with the default rmw stack negotiates PLAIN_CDR2 over the
	// wire — primitives align at most to 4 bytes regardless of the encap
	// representation byte (this matches cdrReader's maxAlign=4 used in
	// DeserializeCDR). With maxAlign=8 here the writer pushed leading float64
	// fields one 8-byte slot too far, so e.g. SetDockingPoint's Pose decoded
	// to garbage on the C++ side because position.x landed at byte 8 while
	// rmw was reading it at byte 4.
	w := &cdrWriter{maxAlign: 4}
	// Encapsulation header: representation_id + options. ROS 2 publishers
	// emit 0x0001 0x0000 (CDR_LE) regardless of the actual XCDR variant.
	// Alignment is measured from the start of the payload (the byte AFTER
	// this 4-byte header) — see the relative-alignment logic in cdrWriter.align.
	w.buf = append(w.buf, 0x00, 0x01, 0x00, 0x00)

	if err := w.writeMessage(msg, schema.Fields); err != nil {
		return nil, err
	}

	// Some CDR implementations (including foxglove_bridge) crash when the
	// payload after the encapsulation header is completely empty (zero-field
	// messages like std_srvs/Trigger request). Append a padding byte to
	// ensure at least 1 byte of body.
	if len(w.buf) == cdrEncapHeaderLen {
		w.buf = append(w.buf, 0x00)
	}

	// Pad the total wire buffer up to a multiple of XCDR1's largest natural
	// alignment unit (8 bytes — float64/uint64). foxglove_bridge's rmw layer
	// rejects requests with `rmw_serialize: invalid data size` when the total
	// length isn't a multiple of 8: e.g. CalibrateImuYaw (a single float64,
	// 4-byte header + 8 data = 12 bytes total) was rejected because rmw
	// expected 16. Nested-struct messages like SetDockingPoint (4 header +
	// 7×float64 = 60) happened to be over the threshold and got through the
	// gate by luck. Padding the buffer to the next multiple of 8 makes both
	// shapes pass the size check; trailing zero bytes are never read by the
	// receiver-side schema walker.
	const wireAlign = 8
	if rem := len(w.buf) % wireAlign; rem != 0 {
		pad := wireAlign - rem
		for i := 0; i < pad; i++ {
			w.buf = append(w.buf, 0)
		}
	}

	return w.buf, nil
}

// cdrEncapHeaderLen is the size of the 4-byte CDR encapsulation header
// (representation_id + options) that precedes the payload.
const cdrEncapHeaderLen = 4

type cdrWriter struct {
	buf      []byte
	maxAlign int
}

// align pads the buffer so the next write lands on an n-byte boundary
// MEASURED FROM THE START OF THE PAYLOAD (i.e. after the 4-byte encapsulation
// header), as required by CDR. Including the header in the modulo would push
// every 8-byte-aligned field 4 bytes too far — fine for messages whose first
// field is a string/uint32 (4-byte alignment is a no-op), but corrupts services
// whose first field needs 8-byte alignment (e.g. SetDockingPoint with a Pose
// containing leading float64s — the receiver reads at relative offset 0 and
// gets garbage from the padding bytes).
func (w *cdrWriter) align(n int) {
	if n <= 1 {
		return
	}
	if n > w.maxAlign {
		n = w.maxAlign
	}
	rem := (len(w.buf) - cdrEncapHeaderLen) % n
	if rem != 0 {
		pad := n - rem
		for i := 0; i < pad; i++ {
			w.buf = append(w.buf, 0)
		}
	}
}

func (w *cdrWriter) writeMessage(msg map[string]interface{}, fields []schemaField) error {
	for _, f := range fields {
		val := msg[f.Name]
		if err := w.writeField(val, f); err != nil {
			return fmt.Errorf("field %s: %w", f.Name, err)
		}
	}
	return nil
}

func (w *cdrWriter) writeField(val interface{}, f schemaField) error {
	switch f.Kind {
	case kindPrimitive:
		return w.writePrimitive(val, f.Primitive)
	case kindString:
		s, _ := val.(string)
		return w.writeString(s)
	case kindMessage:
		m, ok := val.(map[string]interface{})
		if !ok {
			m = make(map[string]interface{})
		}
		return w.writeMessage(m, f.SubFields)
	case kindFixedArray:
		return w.writeArray(val, f, f.ArrayLen)
	case kindDynArray:
		arr := toSlice(val)
		w.align(4)
		w.writeUint32(uint32(len(arr)))
		return w.writeArraySlice(arr, f)
	default:
		return fmt.Errorf("unknown field kind %d", f.Kind)
	}
}

func (w *cdrWriter) writeArray(val interface{}, f schemaField, count int) error {
	arr := toSlice(val)
	// Pad to count if needed
	for len(arr) < count {
		arr = append(arr, nil)
	}
	return w.writeArraySlice(arr[:count], f)
}

func (w *cdrWriter) writeArraySlice(arr []interface{}, f schemaField) error {
	baseType := f.Primitive
	if baseType == "string" {
		for _, v := range arr {
			s, _ := v.(string)
			if err := w.writeString(s); err != nil {
				return err
			}
		}
		return nil
	}
	if isPrimitive(baseType) {
		sz := primitiveSize(baseType)
		w.align(sz)
		for _, v := range arr {
			if err := w.writePrimitive(v, baseType); err != nil {
				return err
			}
		}
		return nil
	}
	// Array of messages
	for _, v := range arr {
		m, ok := v.(map[string]interface{})
		if !ok {
			m = make(map[string]interface{})
		}
		if err := w.writeMessage(m, f.SubFields); err != nil {
			return err
		}
	}
	return nil
}

func (w *cdrWriter) writePrimitive(val interface{}, typ string) error {
	switch typ {
	case "bool":
		w.buf = append(w.buf, boolByte(val))
	case "byte", "uint8":
		w.buf = append(w.buf, uint8(toFloat64(val)))
	case "char", "int8":
		w.buf = append(w.buf, byte(int8(toFloat64(val))))
	case "uint16":
		w.align(2)
		w.writeUint16(uint16(toFloat64(val)))
	case "int16":
		w.align(2)
		w.writeUint16(uint16(int16(toFloat64(val))))
	case "uint32":
		w.align(4)
		w.writeUint32(uint32(toFloat64(val)))
	case "int32":
		w.align(4)
		w.writeUint32(uint32(int32(toFloat64(val))))
	case "uint64":
		w.align(8)
		w.writeUint64(uint64(toFloat64(val)))
	case "int64":
		w.align(8)
		w.writeUint64(uint64(int64(toFloat64(val))))
	case "float32":
		w.align(4)
		w.writeUint32(math.Float32bits(float32(toFloat64(val))))
	case "float64":
		w.align(8)
		w.writeUint64(math.Float64bits(toFloat64(val)))
	default:
		return fmt.Errorf("cdr write: unknown primitive %q", typ)
	}
	return nil
}

func (w *cdrWriter) writeString(s string) error {
	w.align(4)
	w.writeUint32(uint32(len(s) + 1)) // include null terminator
	w.buf = append(w.buf, []byte(s)...)
	w.buf = append(w.buf, 0) // null terminator
	return nil
}

func (w *cdrWriter) writeUint16(v uint16) {
	b := make([]byte, 2)
	binary.LittleEndian.PutUint16(b, v)
	w.buf = append(w.buf, b...)
}

func (w *cdrWriter) writeUint32(v uint32) {
	b := make([]byte, 4)
	binary.LittleEndian.PutUint32(b, v)
	w.buf = append(w.buf, b...)
}

func (w *cdrWriter) writeUint64(v uint64) {
	b := make([]byte, 8)
	binary.LittleEndian.PutUint64(b, v)
	w.buf = append(w.buf, b...)
}

func toFloat64(v interface{}) float64 {
	switch n := v.(type) {
	case float64:
		return n
	case float32:
		return float64(n)
	case int:
		return float64(n)
	case int32:
		return float64(n)
	case int64:
		return float64(n)
	case uint32:
		return float64(n)
	case uint64:
		return float64(n)
	case json.Number:
		f, _ := n.Float64()
		return f
	case bool:
		if n {
			return 1
		}
		return 0
	default:
		return 0
	}
}

func toSlice(v interface{}) []interface{} {
	if v == nil {
		return nil
	}
	if arr, ok := v.([]interface{}); ok {
		return arr
	}
	return nil
}

func boolByte(v interface{}) byte {
	switch b := v.(type) {
	case bool:
		if b {
			return 1
		}
		return 0
	case float64:
		if b != 0 {
			return 1
		}
		return 0
	default:
		return 0
	}
}
