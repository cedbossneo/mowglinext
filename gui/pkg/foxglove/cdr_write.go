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

	w := &cdrWriter{maxAlign: 8} // Classic XCDR1: 8-byte max alignment
	// Encapsulation header: 0x0001 = CDR_LE (classic XCDR1 little-endian).
	// This is ROS 2's default wire encoding for services and messages across
	// all rmw implementations (FastDDS / CycloneDDS), so it's the most
	// compatible choice. Previously we wrote this header with maxAlign=4 —
	// half-XCDR2 — and rmw rejected every request with
	// "rmw_serialize: invalid data size" because float64 landed at offset 4
	// instead of 8. maxAlign=8 fixes float64/uint64 alignment to match.
	w.buf = append(w.buf, 0x00, 0x01, 0x00, 0x00)

	if err := w.writeMessage(msg, schema.Fields); err != nil {
		return nil, err
	}

	// Some CDR implementations (including foxglove_bridge) crash when the
	// payload after the encapsulation header is completely empty (zero-field
	// messages like std_srvs/Trigger request). Append a padding byte to
	// ensure at least 1 byte of body.
	if len(w.buf) == 4 {
		w.buf = append(w.buf, 0x00)
	}

	return w.buf, nil
}

type cdrWriter struct {
	buf      []byte
	maxAlign int
}

func (w *cdrWriter) align(n int) {
	if n <= 1 {
		return
	}
	if n > w.maxAlign {
		n = w.maxAlign
	}
	// Alignment is relative to the encapsulation body start (the byte after
	// the 4-byte CDR header). Mirrors cdrReader.align in cdr.go.
	bodyOff := len(w.buf) - 4
	rem := bodyOff % n
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
