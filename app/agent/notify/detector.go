package notify

import (
	"encoding/base64"
	"encoding/json"
	"log"
	"strings"
	"time"

	"github.com/cedbossneo/mowglinext/app/agent/tunnel"
	"github.com/gorilla/websocket"
)

// HighLevelStatus mirrors the ROS HighLevelStatus type
type HighLevelStatus struct {
	State             int     `json:"State"`
	StateName         string  `json:"StateName"`
	GpsQualityPercent float64 `json:"GpsQualityPercent"`
	BatteryPercent    float64 `json:"BatteryPercent"`
	IsCharging        bool    `json:"IsCharging"`
	Emergency         bool    `json:"Emergency"`
}

// Power mirrors the ROS Power type
type Power struct {
	VBattery      float64 `json:"VBattery"`
	VCharge       float64 `json:"VCharge"`
	ChargeCurrent float64 `json:"ChargeCurrent"`
}

// Emergency mirrors the ROS Emergency type
type Emergency struct {
	ActiveEmergency  bool   `json:"ActiveEmergency"`
	LatchedEmergency bool   `json:"LatchedEmergency"`
	Reason           string `json:"Reason"`
}

// FrameSender can send frames through the tunnel
type FrameSender interface {
	SendFrame(frame *tunnel.TunnelFrame) error
}

// Detector monitors local ROS topics and emits notification events
type Detector struct {
	localURL       string
	sender         FrameSender
	lastState      string
	lastEmergency  bool
	lowBatterySent bool
	gpsLostSent    bool
}

func NewDetector(localURL string, sender FrameSender) *Detector {
	return &Detector{
		localURL: localURL,
		sender:   sender,
	}
}

func (d *Detector) Start() {
	// Monitor multiple topics in parallel
	go d.monitorTopic("highLevelStatus", d.handleHighLevelStatus)
	go d.monitorTopic("power", d.handlePower)
	go d.monitorTopic("emergency", d.handleEmergency)
}

func (d *Detector) monitorTopic(topic string, handler func([]byte)) {
	wsURL := strings.Replace(d.localURL, "http://", "ws://", 1)
	wsURL = strings.Replace(wsURL, "https://", "wss://", 1)
	wsURL += "/api/openmower/subscribe/" + topic

	for {
		conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
		if err != nil {
			log.Printf("Notification detector: failed to connect to %s: %v", topic, err)
			time.Sleep(5 * time.Second)
			continue
		}

		for {
			_, msg, err := conn.ReadMessage()
			if err != nil {
				log.Printf("Notification detector: read error on %s: %v", topic, err)
				conn.Close()
				break
			}

			// Messages are base64 encoded
			decoded, err := base64.StdEncoding.DecodeString(string(msg))
			if err != nil {
				// Try as raw JSON
				decoded = msg
			}

			handler(decoded)
		}

		time.Sleep(2 * time.Second) // Reconnect delay
	}
}

func (d *Detector) handleHighLevelStatus(data []byte) {
	var status HighLevelStatus
	if err := json.Unmarshal(data, &status); err != nil {
		return
	}

	// Mowing complete: MOWING -> IDLE without error
	if d.lastState == "MOWING" && status.StateName == "IDLE" {
		d.emit(&NotificationEvent{
			Type:    EventMowingComplete,
			Title:   "Mowing Complete",
			Body:    "Your mower has finished mowing and returned home.",
			Urgency: "normal",
		})
	}

	// Error state
	if status.StateName == "ERROR" && d.lastState != "ERROR" {
		d.emit(&NotificationEvent{
			Type:    EventError,
			Title:   "Mower Error",
			Body:    "Your mower has encountered an error.",
			Urgency: "high",
		})
	}

	// GPS lost while mowing
	if status.StateName == "MOWING" && status.GpsQualityPercent < 0.1 && !d.gpsLostSent {
		d.gpsLostSent = true
		d.emit(&NotificationEvent{
			Type:    EventGPSLost,
			Title:   "GPS Signal Lost",
			Body:    "GPS quality is very low while mowing. The mower may stop.",
			Urgency: "high",
		})
	} else if status.GpsQualityPercent >= 0.1 {
		d.gpsLostSent = false
	}

	d.lastState = status.StateName
}

func (d *Detector) handlePower(data []byte) {
	var power Power
	if err := json.Unmarshal(data, &power); err != nil {
		return
	}

	// Low battery: VBattery < empty_voltage + 1V (assume empty=23.0V)
	emptyVoltage := 23.0
	if power.VBattery > 0 && power.VBattery < emptyVoltage+1.0 && !d.lowBatterySent {
		d.lowBatterySent = true
		d.emit(&NotificationEvent{
			Type:    EventLowBattery,
			Title:   "Low Battery",
			Body:    "Battery voltage is critically low. The mower should return to dock.",
			Urgency: "high",
		})
	} else if power.VBattery >= emptyVoltage+2.0 {
		d.lowBatterySent = false // Reset with hysteresis
	}
}

func (d *Detector) handleEmergency(data []byte) {
	var emergency Emergency
	if err := json.Unmarshal(data, &emergency); err != nil {
		return
	}

	if emergency.ActiveEmergency && !d.lastEmergency {
		reason := emergency.Reason
		if reason == "" {
			reason = "Unknown reason"
		}
		d.emit(&NotificationEvent{
			Type:    EventEmergency,
			Title:   "Emergency Stop",
			Body:    "Emergency stop activated: " + reason,
			Urgency: "high",
		})
	}

	d.lastEmergency = emergency.ActiveEmergency
}

func (d *Detector) emit(event *NotificationEvent) {
	if d.sender == nil {
		return
	}

	data, err := json.Marshal(event)
	if err != nil {
		return
	}

	frame := &tunnel.TunnelFrame{
		Type: "notification",
		Body: base64.StdEncoding.EncodeToString(data),
	}
	d.sender.SendFrame(frame)
}
