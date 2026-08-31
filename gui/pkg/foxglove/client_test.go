package foxglove

import (
	"encoding/json"
	"math"
	"reflect"
	"testing"
)

func TestSanitizeJSONValueReplacesNonFiniteFloats(t *testing.T) {
	raw := map[string]interface{}{
		"finite": 1.5,
		"nan":    math.NaN(),
		"nested": map[string]interface{}{
			"inf": math.Inf(1),
		},
		"array": []float64{2.0, math.NaN(), 4.0},
	}

	sanitized := sanitizeJSONValue(raw)
	data, err := json.Marshal(sanitized)
	if err != nil {
		t.Fatalf("json.Marshal(sanitizeJSONValue(...)) = %v", err)
	}

	var decoded map[string]interface{}
	if err := json.Unmarshal(data, &decoded); err != nil {
		t.Fatalf("json.Unmarshal = %v", err)
	}

	if decoded["nan"] != nil {
		t.Fatalf("nan field = %#v, want nil", decoded["nan"])
	}

	nested, ok := decoded["nested"].(map[string]interface{})
	if !ok {
		t.Fatalf("nested field type = %T, want map[string]interface{}", decoded["nested"])
	}
	if nested["inf"] != nil {
		t.Fatalf("nested.inf = %#v, want nil", nested["inf"])
	}

	array, ok := decoded["array"].([]interface{})
	if !ok {
		t.Fatalf("array field type = %T, want []interface{}", decoded["array"])
	}
	if len(array) != 3 {
		t.Fatalf("array len = %d, want 3", len(array))
	}
	if array[1] != nil {
		t.Fatalf("array[1] = %#v, want nil", array[1])
	}
}

func TestConnectionStateListenersReceiveDeduplicatedGenerations(t *testing.T) {
	client := NewClient("ws://unused")
	var got []ConnectionState
	remove := client.OnConnectionStateChange(func(state ConnectionState) {
		got = append(got, state)
	})

	client.setConnected(true)
	client.setConnected(true)
	client.setConnected(false)
	client.setConnected(false)
	client.setConnected(true)

	want := []ConnectionState{
		{Connected: true, Generation: 1},
		{Connected: false, Generation: 1},
		{Connected: true, Generation: 2},
	}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("connection events = %#v, want %#v", got, want)
	}

	remove()
	client.setConnected(false)
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("removed listener received another event: %#v", got)
	}
}

func TestConnectionStateListenerCanQueueReentrantTransition(t *testing.T) {
	client := NewClient("ws://unused")
	var got []ConnectionState
	client.OnConnectionStateChange(func(state ConnectionState) {
		got = append(got, state)
		if state.Connected {
			client.setConnected(false)
		}
	})

	client.setConnected(true)

	want := []ConnectionState{
		{Connected: true, Generation: 1},
		{Connected: false, Generation: 1},
	}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("connection events = %#v, want %#v", got, want)
	}
}
