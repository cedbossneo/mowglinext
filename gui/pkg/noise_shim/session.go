package noise_shim

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/flynn/noise"
	"github.com/fxamacker/cbor/v2"
	"github.com/gorilla/websocket"
	"github.com/sirupsen/logrus"
)

// session holds per-connection state for the post-handshake dispatch loop.
type session struct {
	log    *logrus.Entry
	ctx    context.Context
	cancel func()
	conn   *websocket.Conn

	enc, dec *noise.CipherState

	proxy Proxy

	// writeMu serializes ws writes to conn (gorilla/websocket requires this).
	writeMu sync.Mutex

	// outboundCh carries fully-encoded plaintext frames to a single writer
	// goroutine. Buffered to absorb bursty subscriptions.
	outboundCh chan []byte

	wsMu sync.Mutex
	ws   map[uint32]WSStream
}

func (s *session) run() error {
	// Wrap parent ctx so cancel() also signals our writerLoop and any
	// in-flight sendFrame callers. Closing the WS conn alone is not enough
	// because background goroutines (handleWSOpen reader) keep producing
	// frames after readerLoop exits.
	wrapCtx, wrapCancel := context.WithCancel(s.ctx)
	s.ctx = wrapCtx
	defer wrapCancel()
	defer s.cancel()
	defer s.closeAllWS()

	s.outboundCh = make(chan []byte, 64)
	wg := sync.WaitGroup{}

	wg.Add(1)
	go func() {
		defer wg.Done()
		s.writerLoop()
	}()

	err := s.readerLoop()
	wrapCancel() // signal writerLoop + sendFrame to stop
	wg.Wait()
	return err
}

func (s *session) writerLoop() {
	for {
		select {
		case <-s.ctx.Done():
			return
		case plain, ok := <-s.outboundCh:
			if !ok {
				return
			}
			ct, err := s.enc.Encrypt(nil, nil, plain)
			if err != nil {
				s.log.WithError(err).Warn("encrypt failed")
				s.cancel()
				return
			}
			framed := make([]byte, 2+len(ct))
			binary.BigEndian.PutUint16(framed[:2], uint16(len(ct)))
			copy(framed[2:], ct)
			s.writeMu.Lock()
			err = s.conn.WriteMessage(websocket.BinaryMessage, framed)
			s.writeMu.Unlock()
			if err != nil {
				s.log.WithError(err).Debug("write failed")
				s.cancel()
				return
			}
		}
	}
}

func (s *session) readerLoop() error {
	for {
		select {
		case <-s.ctx.Done():
			return s.ctx.Err()
		default:
		}
		mt, msg, err := s.conn.ReadMessage()
		if err != nil {
			return err
		}
		if mt != websocket.BinaryMessage {
			continue
		}
		if len(msg) < 2 {
			return errors.New("frame too short")
		}
		declared := binary.BigEndian.Uint16(msg[:2])
		if int(declared) != len(msg)-2 {
			return errors.New("frame length mismatch")
		}
		plain, err := s.dec.Decrypt(nil, nil, msg[2:])
		if err != nil {
			return fmt.Errorf("decrypt: %w", err)
		}
		if len(plain) > MaxFrameSize {
			return errors.New("frame too large")
		}
		var f Frame
		if err := cbor.Unmarshal(plain, &f); err != nil {
			s.log.WithError(err).Warn("cbor decode")
			continue
		}
		s.dispatch(f)
	}
}

func (s *session) sendFrame(f Frame) {
	b, err := cbor.Marshal(f)
	if err != nil {
		s.log.WithError(err).Warn("encode frame")
		return
	}
	select {
	case s.outboundCh <- b:
	case <-s.ctx.Done():
	}
}

func (s *session) closeAllWS() {
	s.wsMu.Lock()
	defer s.wsMu.Unlock()
	for id, w := range s.ws {
		_ = w.Close(websocket.CloseGoingAway, "session ended")
		delete(s.ws, id)
	}
}

func (s *session) dispatch(f Frame) {
	switch f.Type {
	case FrameReq:
		go s.handleReq(f)
	case FrameWSOpen:
		go s.handleWSOpen(f)
	case FrameWSData:
		s.handleWSData(f)
	case FrameWSClose:
		s.handleWSClose(f)
	case FramePing:
		s.sendFrame(Frame{Type: FramePong, ID: f.ID, TS: time.Now().UnixMilli()})
	default:
		s.log.WithField("type", f.Type).Debug("unknown frame")
	}
}

func (s *session) handleReq(f Frame) {
	ctx, cancel := context.WithTimeout(s.ctx, 30*time.Second)
	defer cancel()
	status, h, body, err := s.proxy.HTTP(ctx, f.Method, f.Path, f.Headers, f.Body)
	out := Frame{Type: FrameRes, ID: f.ID}
	if err != nil {
		out.Status = 502
		out.Body = []byte(err.Error())
	} else {
		out.Status = status
		out.Headers = h
		out.Body = body
	}
	s.sendFrame(out)
}

func (s *session) handleWSOpen(f Frame) {
	stream, err := s.proxy.OpenWS(s.ctx, f.Path, f.Headers)
	if err != nil {
		s.sendFrame(Frame{Type: FrameWSErr, ID: f.ID, Reason: err.Error()})
		return
	}
	s.wsMu.Lock()
	s.ws[f.ID] = stream
	s.wsMu.Unlock()
	s.sendFrame(Frame{Type: FrameWSAck, ID: f.ID})

	go func() {
		defer func() {
			s.wsMu.Lock()
			delete(s.ws, f.ID)
			s.wsMu.Unlock()
			_ = stream.Close(websocket.CloseNormalClosure, "")
		}()
		for {
			isBin, data, err := stream.ReadFrame()
			if err != nil {
				s.sendFrame(Frame{Type: FrameWSClose, ID: f.ID, Code: 1000, Reason: err.Error()})
				return
			}
			s.sendFrame(Frame{Type: FrameWSData, ID: f.ID, Data: data, Binary: isBin})
		}
	}()
}

func (s *session) handleWSData(f Frame) {
	s.wsMu.Lock()
	stream, ok := s.ws[f.ID]
	s.wsMu.Unlock()
	if !ok {
		return
	}
	if err := stream.WriteFrame(f.Binary, f.Data); err != nil {
		s.sendFrame(Frame{Type: FrameWSClose, ID: f.ID, Code: 1011, Reason: err.Error()})
	}
}

func (s *session) handleWSClose(f Frame) {
	s.wsMu.Lock()
	stream, ok := s.ws[f.ID]
	if ok {
		delete(s.ws, f.ID)
	}
	s.wsMu.Unlock()
	if ok {
		_ = stream.Close(f.Code, f.Reason)
	}
}
