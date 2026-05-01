package noise_shim

import (
	"context"
	"crypto/rand"
	"encoding/binary"
	"errors"
	"net/http/httptest"
	"net/url"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/flynn/noise"
	"github.com/fxamacker/cbor/v2"
	"github.com/gorilla/websocket"
	"github.com/sirupsen/logrus"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type fakeProxy struct {
	mu      sync.Mutex
	httpLog []string
	respond func(method, path string) (int, map[string]string, []byte, error)
	openWS  func(path string) (WSStream, error)
}

func (f *fakeProxy) HTTP(_ context.Context, method, path string, _ map[string]string, _ []byte) (int, map[string]string, []byte, error) {
	f.mu.Lock()
	f.httpLog = append(f.httpLog, method+" "+path)
	f.mu.Unlock()
	if f.respond != nil {
		return f.respond(method, path)
	}
	return 200, map[string]string{"Content-Type": "application/json"}, []byte(`{"ok":true}`), nil
}

func (f *fakeProxy) OpenWS(_ context.Context, path string, _ map[string]string) (WSStream, error) {
	if f.openWS == nil {
		return nil, errors.New("no fake openWS")
	}
	return f.openWS(path)
}

type fakeWSStream struct {
	mu      sync.Mutex
	closed  bool
	read    chan fakeFrame
	write   func(binary bool, data []byte) error
	onClose func()
}

type fakeFrame struct {
	binary bool
	data   []byte
	err    error
}

func newFakeWSStream() *fakeWSStream {
	return &fakeWSStream{read: make(chan fakeFrame, 16)}
}

func (s *fakeWSStream) WriteFrame(binary bool, data []byte) error {
	if s.write != nil {
		return s.write(binary, data)
	}
	return nil
}

func (s *fakeWSStream) ReadFrame() (bool, []byte, error) {
	f, ok := <-s.read
	if !ok {
		return false, nil, errors.New("closed")
	}
	return f.binary, f.data, f.err
}

func (s *fakeWSStream) Close(_ int, _ string) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed {
		return nil
	}
	s.closed = true
	close(s.read)
	if s.onClose != nil {
		s.onClose()
	}
	return nil
}

func TestRoundTrip(t *testing.T) {
	logrus.SetLevel(logrus.PanicLevel)

	robotKP, err := noise.DH25519.GenerateKeypair(rand.Reader)
	require.NoError(t, err)

	allow := NewMemoryAllowList([]string{"alice"})
	ver := StaticVerifier{UID: "alice"}

	upstream := newFakeWSStream()
	proxy := &fakeProxy{
		openWS: func(path string) (WSStream, error) {
			require.Equal(t, "/api/mowglinext/subscribe/gps", path)
			return upstream, nil
		},
	}

	srv := New(Config{
		StaticPriv: robotKP.Private,
		RobotID:    "test-rid",
		RobotName:  "Lawn Bot",
		Version:    "0.0.1",
		Allow:      allow,
		Verifier:   ver,
		Proxy:      proxy,
	})

	hs := httptest.NewServer(srv.Handler())
	defer hs.Close()

	wsURL := "ws" + strings.TrimPrefix(hs.URL, "http")
	u, err := url.Parse(wsURL)
	require.NoError(t, err)

	c, _, err := websocket.DefaultDialer.Dial(u.String(), nil)
	require.NoError(t, err)
	defer c.Close()

	cs := noise.NewCipherSuite(noise.DH25519, noise.CipherChaChaPoly, noise.HashBLAKE2s)
	clientKP, err := noise.DH25519.GenerateKeypair(rand.Reader)
	require.NoError(t, err)
	hsClient, err := noise.NewHandshakeState(noise.Config{
		CipherSuite:   cs,
		Pattern:       noise.HandshakeIK,
		Initiator:     true,
		StaticKeypair: clientKP,
		PeerStatic:    robotKP.Public,
	})
	require.NoError(t, err)

	nonce := make([]byte, 32)
	_, _ = rand.Read(nonce)
	payload := HandshakePayload{
		FirebaseIDToken: "fake-token-ok",
		Nonce:           nonce,
		AppVersion:      "test-1",
	}
	pb, err := cbor.Marshal(payload)
	require.NoError(t, err)

	out, _, _, err := hsClient.WriteMessage(nil, pb)
	require.NoError(t, err)
	framed := make([]byte, 2+len(out))
	binary.BigEndian.PutUint16(framed[:2], uint16(len(out)))
	copy(framed[2:], out)
	require.NoError(t, c.WriteMessage(websocket.BinaryMessage, framed))

	_, msg2, err := c.ReadMessage()
	require.NoError(t, err)
	require.GreaterOrEqual(t, len(msg2), 2)
	declared := binary.BigEndian.Uint16(msg2[:2])
	require.Equal(t, int(declared), len(msg2)-2)
	hello, csTx, csRx, err := hsClient.ReadMessage(nil, msg2[2:])
	require.NoError(t, err)
	require.NotNil(t, csTx)
	require.NotNil(t, csRx)
	var reply HandshakeReply
	require.NoError(t, cbor.Unmarshal(hello, &reply))
	assert.Equal(t, "test-rid", reply.RobotID)
	assert.Equal(t, "Lawn Bot", reply.RobotName)

	// Noise convention: cs1 is initiator-out (we send with cs1), cs2 is
	// initiator-in (we receive with cs2). flynn/noise returns them in that
	// order from ReadMessage on the initiator.
	clientTx := csTx
	clientRx := csRx

	sendFrame := func(f Frame) error {
		b, err := cbor.Marshal(f)
		if err != nil {
			return err
		}
		ct, err := clientTx.Encrypt(nil, nil, b)
		if err != nil {
			return err
		}
		out := make([]byte, 2+len(ct))
		binary.BigEndian.PutUint16(out[:2], uint16(len(ct)))
		copy(out[2:], ct)
		return c.WriteMessage(websocket.BinaryMessage, out)
	}
	recvFrame := func() (Frame, error) {
		_, msg, err := c.ReadMessage()
		if err != nil {
			return Frame{}, err
		}
		if len(msg) < 2 {
			return Frame{}, errors.New("short")
		}
		dec := binary.BigEndian.Uint16(msg[:2])
		if int(dec) != len(msg)-2 {
			return Frame{}, errors.New("len mismatch")
		}
		plain, err := clientRx.Decrypt(nil, nil, msg[2:])
		if err != nil {
			return Frame{}, err
		}
		var f Frame
		if err := cbor.Unmarshal(plain, &f); err != nil {
			return Frame{}, err
		}
		return f, nil
	}

	require.NoError(t, sendFrame(Frame{
		Type:   FrameReq,
		ID:     1,
		Method: "GET",
		Path:   "/api/system/info",
	}))
	resp, err := recvFrame()
	require.NoError(t, err)
	assert.Equal(t, FrameRes, resp.Type)
	assert.Equal(t, uint32(1), resp.ID)
	assert.Equal(t, 200, resp.Status)
	assert.JSONEq(t, `{"ok":true}`, string(resp.Body))

	require.NoError(t, sendFrame(Frame{
		Type: FrameWSOpen,
		ID:   42,
		Path: "/api/mowglinext/subscribe/gps",
	}))
	ack, err := recvFrame()
	require.NoError(t, err)
	require.Equal(t, FrameWSAck, ack.Type)
	require.Equal(t, uint32(42), ack.ID)

	upstream.read <- fakeFrame{binary: false, data: []byte(`{"lat":1,"lon":2}`)}
	got, err := recvFrame()
	require.NoError(t, err)
	assert.Equal(t, FrameWSData, got.Type)
	assert.Equal(t, uint32(42), got.ID)
	assert.JSONEq(t, `{"lat":1,"lon":2}`, string(got.Data))

	require.NoError(t, sendFrame(Frame{Type: FrameWSClose, ID: 42, Code: 1000}))

	proxy.mu.Lock()
	defer proxy.mu.Unlock()
	require.Contains(t, proxy.httpLog, "GET /api/system/info")
}

func TestNonceReplay(t *testing.T) {
	srv := New(Config{
		StaticPriv: make([]byte, 32),
		Allow:      NewMemoryAllowList([]string{"x"}),
		Verifier:   StaticVerifier{UID: "x"},
		Proxy:      &fakeProxy{},
	})
	n := []byte("0123456789abcdef")
	require.NoError(t, srv.checkNonce(n))
	err := srv.checkNonce(n)
	require.Error(t, err)
	assert.Contains(t, err.Error(), "replayed")
}

func TestNonceWindowEviction(t *testing.T) {
	srv := New(Config{
		StaticPriv: make([]byte, 32),
		Allow:      NewMemoryAllowList([]string{"x"}),
		Verifier:   StaticVerifier{UID: "x"},
		Proxy:      &fakeProxy{},
	})
	n := []byte("0123456789abcdef")
	require.NoError(t, srv.checkNonce(n))
	srv.noncesMu.Lock()
	srv.recentNonces["nonce-key"] = time.Now().Add(-2 * NonceWindow)
	srv.noncesMu.Unlock()
	require.NoError(t, srv.checkNonce([]byte("zzzzzzzzzzzzzzzz")))
	srv.noncesMu.Lock()
	_, stillThere := srv.recentNonces["nonce-key"]
	srv.noncesMu.Unlock()
	assert.False(t, stillThere)
}

func TestAllowListReplace(t *testing.T) {
	a := NewMemoryAllowList([]string{"u1", "u2"})
	assert.True(t, a.Contains("u1"))
	a.Replace([]string{"u3"})
	assert.False(t, a.Contains("u1"))
	assert.True(t, a.Contains("u3"))
	snap := a.Snapshot()
	assert.ElementsMatch(t, []string{"u3"}, snap)
}
