package noise_shim

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"strings"
	"time"

	"github.com/gorilla/websocket"
)

// LocalHTTPProxy implements Proxy by forwarding to "http://localhost:<port>".
// It is the default Proxy used by the shim in production. Tests use a fake.
type LocalHTTPProxy struct {
	BaseURL string // e.g. "http://127.0.0.1:4006"
	Client  *http.Client
	Dialer  *websocket.Dialer
}

func NewLocalHTTPProxy(baseURL string) *LocalHTTPProxy {
	return &LocalHTTPProxy{
		BaseURL: strings.TrimRight(baseURL, "/"),
		Client:  &http.Client{Timeout: 30 * time.Second},
		Dialer:  websocket.DefaultDialer,
	}
}

func (p *LocalHTTPProxy) HTTP(ctx context.Context, method, path string, headers map[string]string, body []byte) (int, map[string]string, []byte, error) {
	if !strings.HasPrefix(path, "/") {
		return 0, nil, nil, errors.New("path must start with /")
	}
	req, err := http.NewRequestWithContext(ctx, method, p.BaseURL+path, bytes.NewReader(body))
	if err != nil {
		return 0, nil, nil, err
	}
	for k, v := range headers {
		if isHopByHop(k) {
			continue
		}
		req.Header.Set(k, v)
	}
	resp, err := p.Client.Do(req)
	if err != nil {
		return 0, nil, nil, err
	}
	defer resp.Body.Close()
	respBody, err := io.ReadAll(resp.Body)
	if err != nil {
		return 0, nil, nil, err
	}
	respHeaders := make(map[string]string, len(resp.Header))
	for k, vs := range resp.Header {
		if isHopByHop(k) {
			continue
		}
		respHeaders[k] = strings.Join(vs, ", ")
	}
	return resp.StatusCode, respHeaders, respBody, nil
}

func (p *LocalHTTPProxy) OpenWS(ctx context.Context, path string, headers map[string]string) (WSStream, error) {
	if !strings.HasPrefix(path, "/") {
		return nil, errors.New("path must start with /")
	}
	u, err := url.Parse(p.BaseURL + path)
	if err != nil {
		return nil, err
	}
	switch u.Scheme {
	case "http":
		u.Scheme = "ws"
	case "https":
		u.Scheme = "wss"
	default:
		return nil, fmt.Errorf("unsupported scheme: %s", u.Scheme)
	}
	hdr := http.Header{}
	for k, v := range headers {
		if isHopByHop(k) {
			continue
		}
		hdr.Set(k, v)
	}
	conn, _, err := p.Dialer.DialContext(ctx, u.String(), hdr)
	if err != nil {
		return nil, err
	}
	return &gorillaWSStream{conn: conn}, nil
}

type gorillaWSStream struct {
	conn *websocket.Conn
}

func (g *gorillaWSStream) WriteFrame(binary bool, data []byte) error {
	mt := websocket.TextMessage
	if binary {
		mt = websocket.BinaryMessage
	}
	return g.conn.WriteMessage(mt, data)
}

func (g *gorillaWSStream) ReadFrame() (bool, []byte, error) {
	mt, data, err := g.conn.ReadMessage()
	return mt == websocket.BinaryMessage, data, err
}

func (g *gorillaWSStream) Close(code int, reason string) error {
	_ = g.conn.WriteControl(
		websocket.CloseMessage,
		websocket.FormatCloseMessage(code, reason),
		time.Now().Add(time.Second),
	)
	return g.conn.Close()
}

var hopByHop = map[string]struct{}{
	"connection":          {},
	"keep-alive":          {},
	"proxy-authenticate":  {},
	"proxy-authorization": {},
	"te":                  {},
	"trailer":             {},
	"transfer-encoding":   {},
	"upgrade":             {},
	"host":                {},
	"content-length":      {},
}

func isHopByHop(k string) bool {
	_, ok := hopByHop[strings.ToLower(k)]
	return ok
}
