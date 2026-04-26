package api

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"net/http"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

// statusTopic is the logical key registered in providers.topicMap for the
// CalibrateImuYawStatus topic — NOT the raw ROS topic name. The provider
// translates this to /calibrate_imu_yaw_node/calibrate_status and creates a
// foxglove_bridge subscription. See mowgli_interfaces/msg/CalibrateImuYawStatus
// and the discussion in mowgli_interfaces/srv/CalibrateImuYaw for why the
// result moved off the service response.
const statusTopic = "calibrateStatus"

// ---------------------------------------------------------------------------
// Request / Response types
// ---------------------------------------------------------------------------

// CalibrateImuYawRequest is the JSON body for both the legacy synchronous
// POST /calibration/imu-yaw endpoint and the new POST /calibration/imu-yaw/start.
type CalibrateImuYawRequest struct {
	DurationSec float64 `json:"duration_sec"`
}

// CalibrateImuYawResponse mirrors the ROS service response 1:1.
type CalibrateImuYawResponse struct {
	Success               bool    `json:"success"`
	Message               string  `json:"message"`
	ImuYawRad             float64 `json:"imu_yaw_rad"`
	ImuYawDeg             float64 `json:"imu_yaw_deg"`
	SamplesUsed           int32   `json:"samples_used"`
	StdDevDeg             float64 `json:"std_dev_deg"`
	ImuPitchRad           float64 `json:"imu_pitch_rad"`
	ImuPitchDeg           float64 `json:"imu_pitch_deg"`
	ImuRollRad            float64 `json:"imu_roll_rad"`
	ImuRollDeg            float64 `json:"imu_roll_deg"`
	StationarySamplesUsed int32   `json:"stationary_samples_used"`
	GravityMagMps2        float64 `json:"gravity_mag_mps2"`
}

// CalibrationJobState enumerates the lifecycle of a long-running calibration.
type CalibrationJobState string

const (
	CalibrationJobRunning CalibrationJobState = "running"
	CalibrationJobDone    CalibrationJobState = "done"
	CalibrationJobFailed  CalibrationJobState = "failed"
)

// CalibrationJob is the API-facing view of a queued IMU-yaw calibration.
type CalibrationJob struct {
	ID        string                   `json:"id"`
	State     CalibrationJobState      `json:"state"`
	StartedAt time.Time                `json:"started_at"`
	EndedAt   *time.Time               `json:"ended_at,omitempty"`
	Result    *CalibrateImuYawResponse `json:"result,omitempty"`
	Error     string                   `json:"error,omitempty"`
}

// StartCalibrationResponse is returned from POST /calibration/imu-yaw/start.
type StartCalibrationResponse struct {
	JobID     string    `json:"job_id"`
	StartedAt time.Time `json:"started_at"`
}

// ---------------------------------------------------------------------------
// In-memory job store
// ---------------------------------------------------------------------------

// calibrationJobStore is a thread-safe map of in-flight + recently-completed
// calibration jobs. Completed jobs stay around for `retentionWindow` so the
// frontend has time to poll the final result; older entries are pruned on
// every store access. The store deliberately lives only in process memory —
// a calibration in flight does not survive a GUI restart, which matches the
// physical reality that the robot itself stops driving on restart.
type calibrationJobStore struct {
	mu              sync.Mutex
	jobs            map[string]*CalibrationJob
	retentionWindow time.Duration
}

func newCalibrationJobStore() *calibrationJobStore {
	return &calibrationJobStore{
		jobs:            make(map[string]*CalibrationJob),
		retentionWindow: 10 * time.Minute,
	}
}

func (s *calibrationJobStore) create() *CalibrationJob {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pruneLocked()

	job := &CalibrationJob{
		ID:        newJobID(),
		State:     CalibrationJobRunning,
		StartedAt: time.Now().UTC(),
	}
	s.jobs[job.ID] = job
	return job
}

func (s *calibrationJobStore) get(id string) (*CalibrationJob, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pruneLocked()
	job, ok := s.jobs[id]
	return job, ok
}

func (s *calibrationJobStore) markDone(id string, res *CalibrateImuYawResponse) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if job, ok := s.jobs[id]; ok {
		job.State = CalibrationJobDone
		job.Result = res
		now := time.Now().UTC()
		job.EndedAt = &now
	}
}

func (s *calibrationJobStore) markFailed(id string, errMsg string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if job, ok := s.jobs[id]; ok {
		job.State = CalibrationJobFailed
		job.Error = errMsg
		now := time.Now().UTC()
		job.EndedAt = &now
	}
}

// pruneLocked drops jobs whose EndedAt is older than retentionWindow.
// Callers must hold s.mu.
func (s *calibrationJobStore) pruneLocked() {
	cutoff := time.Now().Add(-s.retentionWindow)
	for id, job := range s.jobs {
		if job.EndedAt != nil && job.EndedAt.Before(cutoff) {
			delete(s.jobs, id)
		}
	}
}

func newJobID() string {
	var b [12]byte
	if _, err := rand.Read(b[:]); err != nil {
		// rand.Read is documented to never fail in practice on Linux;
		// fall back to time-based id so we still return *something* unique.
		return time.Now().UTC().Format("20060102T150405.000000000")
	}
	return hex.EncodeToString(b[:])
}

// ---------------------------------------------------------------------------
// Route registration
// ---------------------------------------------------------------------------

// CalibrationRoutes registers sensor-calibration endpoints.
func CalibrationRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider) {
	store := newCalibrationJobStore()
	group := r.Group("/calibration")

	// Async API — preferred. The frontend should start a job and poll status.
	group.POST("/imu-yaw/start", postStartCalibrateImuYaw(rosProvider, store))
	group.GET("/imu-yaw/status/:job_id", getCalibrateImuYawStatus(store))

	// Backwards-compatible synchronous endpoint. Internally starts a job
	// and blocks until it terminates, so foxglove_bridge's per-call timeout
	// no longer races the calibration's runtime — the goroutine owns the
	// long ROS service call, and the HTTP request waits on the local job
	// store. Existing callers see no behavioural change.
	group.POST("/imu-yaw", postCalibrateImuYaw(rosProvider, store))
}

// ---------------------------------------------------------------------------
// POST /calibration/imu-yaw/start
// ---------------------------------------------------------------------------

func postStartCalibrateImuYaw(rosProvider types.IRosProvider, store *calibrationJobStore) gin.HandlerFunc {
	return func(c *gin.Context) {
		body, err := bindCalibrationBody(c)
		if err != nil {
			return // bindCalibrationBody already wrote the 400 response
		}

		job := store.create()
		go runImuYawCalibration(rosProvider, store, job.ID, body.DurationSec)

		c.JSON(http.StatusAccepted, StartCalibrationResponse{
			JobID:     job.ID,
			StartedAt: job.StartedAt,
		})
	}
}

// ---------------------------------------------------------------------------
// GET /calibration/imu-yaw/status/:job_id
// ---------------------------------------------------------------------------

func getCalibrateImuYawStatus(store *calibrationJobStore) gin.HandlerFunc {
	return func(c *gin.Context) {
		id := c.Param("job_id")
		job, ok := store.get(id)
		if !ok {
			c.JSON(http.StatusNotFound, ErrorResponse{Error: "Unknown calibration job: " + id})
			return
		}
		c.JSON(http.StatusOK, job)
	}
}

// ---------------------------------------------------------------------------
// POST /calibration/imu-yaw  (legacy synchronous wrapper)
// ---------------------------------------------------------------------------

// postCalibrateImuYaw kicks off a job and blocks the HTTP request until it
// terminates. Used by older clients that expect the original synchronous
// shape. Returns 200 on success, 500 on calibration failure or timeout.
func postCalibrateImuYaw(rosProvider types.IRosProvider, store *calibrationJobStore) gin.HandlerFunc {
	return func(c *gin.Context) {
		body, err := bindCalibrationBody(c)
		if err != nil {
			return
		}

		job := store.create()
		go runImuYawCalibration(rosProvider, store, job.ID, body.DurationSec)

		// Poll the local store rather than blocking on the ROS service
		// directly. This keeps the HTTP path off the foxglove_bridge
		// service-call timeout entirely.
		timeout := time.Duration(clampDuration(body.DurationSec)+30.0) * time.Second
		ctx, cancel := context.WithTimeout(c.Request.Context(), timeout)
		defer cancel()

		ticker := time.NewTicker(250 * time.Millisecond)
		defer ticker.Stop()

		for {
			select {
			case <-ctx.Done():
				c.JSON(http.StatusGatewayTimeout, ErrorResponse{
					Error: "Calibration did not complete within timeout — poll /calibration/imu-yaw/status/" + job.ID + " for the final result.",
				})
				return
			case <-ticker.C:
				latest, ok := store.get(job.ID)
				if !ok {
					c.JSON(http.StatusInternalServerError, ErrorResponse{
						Error: "Calibration job vanished before completing",
					})
					return
				}
				switch latest.State {
				case CalibrationJobDone:
					if latest.Result == nil {
						c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Calibration finished without a result payload"})
						return
					}
					c.JSON(http.StatusOK, latest.Result)
					return
				case CalibrationJobFailed:
					c.JSON(http.StatusInternalServerError, ErrorResponse{Error: latest.Error})
					return
				}
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------

// runImuYawCalibration triggers the calibrate_imu_yaw_node service (which
// returns immediately with bool success only) and waits for the full result
// to arrive on the status topic. Always called from a fresh goroutine so
// neither the service's foxglove_bridge call nor the status wait blocks the
// HTTP request.
//
// Why the indirection? The service response originally carried 12 mixed-type
// fields (yaw / pitch / roll / sample counts / message). That shape tickled
// a foxglove_bridge ↔ rmw_cyclonedds GenericClient typesupport-dispatch bug
// that made the GUI calibration button silently fail (#19). Splitting the
// result onto a topic — which doesn't go through the same dispatch path —
// is the workaround. job_id matching prevents a stale `transient_local`
// status from a previous run from being mistaken for ours.
func runImuYawCalibration(rosProvider types.IRosProvider, store *calibrationJobStore, jobID string, durationSec float64) {
	// Generous timeout: the calibration drive itself runs ~30 s and we add a
	// margin for foxglove_bridge round-trip + RECORDING/IDLE BT transitions.
	timeout := time.Duration(clampDuration(durationSec)+60.0) * time.Second
	ctx, cancel := context.WithTimeout(context.Background(), timeout)
	defer cancel()

	// Subscribe BEFORE invoking the service — we don't want to race the
	// node's terminal publish.
	statusCh := make(chan *mowgli.CalibrateImuYawStatus, 4)
	subErr := rosProvider.Subscribe(statusTopic, jobID, func(msg []byte) {
		var s mowgli.CalibrateImuYawStatus
		if err := json.Unmarshal(msg, &s); err != nil {
			return // malformed; foxglove_bridge will keep retrying
		}
		// Filter stale/cached statuses from a previous run.
		if s.JobId != jobID {
			return
		}
		select {
		case statusCh <- &s:
		default:
			// Buffer is full (we only need the terminal Done message).
			// Drop intermediates rather than block the bridge thread.
		}
	})
	if subErr != nil {
		store.markFailed(jobID, "Failed to subscribe to calibration status: "+subErr.Error())
		return
	}
	defer rosProvider.UnSubscribe(statusTopic, jobID)

	req := mowgli.CalibrateImuYawReq{
		DurationSec: durationSec,
		JobId:       jobID,
	}
	var res mowgli.CalibrateImuYawRes
	if err := rosProvider.CallService(
		ctx,
		"/calibrate_imu_yaw_node/calibrate",
		&req,
		&res,
		"mowgli_interfaces/srv/CalibrateImuYaw",
	); err != nil {
		store.markFailed(jobID, "Failed to call calibration service: "+err.Error())
		return
	}
	if !res.Success {
		// Preflight rejected the request OR another run is in flight. The
		// node will have published a status with done=true success=false on
		// the same topic; we still wait for it so the user sees the actual
		// reason rather than a generic failure.
	}

	for {
		select {
		case s := <-statusCh:
			if !s.Done {
				continue // progress update; ignore for now
			}
			store.markDone(jobID, &CalibrateImuYawResponse{
				Success:               s.Success,
				Message:               s.Message,
				ImuYawRad:             s.ImuYawRad,
				ImuYawDeg:             s.ImuYawDeg,
				SamplesUsed:           s.SamplesUsed,
				StdDevDeg:             s.StdDevDeg,
				ImuPitchRad:           s.ImuPitchRad,
				ImuPitchDeg:           s.ImuPitchDeg,
				ImuRollRad:            s.ImuRollRad,
				ImuRollDeg:            s.ImuRollDeg,
				StationarySamplesUsed: s.StationarySamplesUsed,
				GravityMagMps2:        s.GravityMagMps2,
			})
			return
		case <-ctx.Done():
			store.markFailed(jobID, "Timed out waiting for calibration status")
			return
		}
	}
}

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

// bindCalibrationBody parses the JSON body and writes a 400 response on
// failure. Returns the parsed body plus a non-nil error if the caller should
// abort.
func bindCalibrationBody(c *gin.Context) (CalibrateImuYawRequest, error) {
	var body CalibrateImuYawRequest
	if err := c.BindJSON(&body); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "Invalid request body: " + err.Error()})
		return body, err
	}
	return body, nil
}

// clampDuration mirrors the bounds the calibrate-imu-yaw ROS node enforces
// internally: 0 means "use the node default" (~30 s), and the node refuses
// anything over 120 s as a safety guard against drive-loop runaway.
func clampDuration(durationSec float64) float64 {
	if durationSec <= 0 {
		return 30.0
	}
	if durationSec > 120.0 {
		return 120.0
	}
	return durationSec
}
