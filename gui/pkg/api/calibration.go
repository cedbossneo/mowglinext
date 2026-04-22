package api

import (
	"context"
	"net/http"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

// ---------------------------------------------------------------------------
// Request / Response types
// ---------------------------------------------------------------------------

// CalibrateImuYawRequest is the JSON body for POST /calibration/imu-yaw.
type CalibrateImuYawRequest struct {
	DurationSec float64 `json:"duration_sec"`
}

// CalibrateImuYawResponse mirrors the ROS service response 1:1.
type CalibrateImuYawResponse struct {
	Success     bool    `json:"success"`
	Message     string  `json:"message"`
	ImuYawRad   float64 `json:"imu_yaw_rad"`
	ImuYawDeg   float64 `json:"imu_yaw_deg"`
	SamplesUsed int32   `json:"samples_used"`
	StdDevDeg   float64 `json:"std_dev_deg"`
}

// ---------------------------------------------------------------------------
// Route registration
// ---------------------------------------------------------------------------

// CalibrationRoutes registers sensor-calibration endpoints.
func CalibrationRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider) {
	group := r.Group("/calibration")
	group.POST("/imu-yaw", postCalibrateImuYaw(rosProvider))
}

// ---------------------------------------------------------------------------
// POST /calibration/imu-yaw
// ---------------------------------------------------------------------------

// postCalibrateImuYaw forwards the request to the ROS service
// `/calibrate_imu_yaw_node/calibrate`. The caller must supply the collection
// window; the ROS service blocks for that duration, so the HTTP timeout is
// set to `duration_sec + 10s`.
func postCalibrateImuYaw(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var body CalibrateImuYawRequest
		if err := c.BindJSON(&body); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "Invalid request body: " + err.Error()})
			return
		}

		// Clamp sane bounds. 0 means "use node default" (30s), so we still
		// budget generously for it.
		duration := body.DurationSec
		if duration <= 0 {
			duration = 30.0
		}
		if duration > 120.0 {
			duration = 120.0
		}

		timeout := time.Duration(duration+10.0) * time.Second
		ctx, cancel := context.WithTimeout(c.Request.Context(), timeout)
		defer cancel()

		req := mowgli.CalibrateImuYawReq{DurationSec: body.DurationSec}
		var res mowgli.CalibrateImuYawRes
		if err := rosProvider.CallService(
			ctx,
			"/calibrate_imu_yaw_node/calibrate",
			&req,
			&res,
			"mowgli_interfaces/srv/CalibrateImuYaw",
		); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{
				Error: "Failed to call calibration service: " + err.Error(),
			})
			return
		}

		c.JSON(http.StatusOK, CalibrateImuYawResponse{
			Success:     res.Success,
			Message:     res.Message,
			ImuYawRad:   res.ImuYawRad,
			ImuYawDeg:   res.ImuYawDeg,
			SamplesUsed: res.SamplesUsed,
			StdDevDeg:   res.StdDevDeg,
		})
	}
}
