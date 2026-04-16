package api

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"gopkg.in/yaml.v3"
)

// ---------------------------------------------------------------------------
// Response types
// ---------------------------------------------------------------------------

// DiagnosticsSnapshot is the top-level response for GET /diagnostics/snapshot.
type DiagnosticsSnapshot struct {
	Timestamp   string             `json:"timestamp"`
	Containers  []ContainerHealth  `json:"containers"`
	System      SystemHealth       `json:"system"`
	Coverage    []AreaCoverageInfo `json:"coverage"`
	CrossChecks CrossChecks        `json:"cross_checks"`
	SlamInfo    SlamInfo           `json:"slam_info"`
}

// ContainerHealth holds the health summary of a single Docker container.
type ContainerHealth struct {
	Name      string `json:"name"`
	State     string `json:"state"`
	Status    string `json:"status"`
	StartedAt string `json:"started_at"`
}

// SystemHealth holds host-level health metrics.
type SystemHealth struct {
	CpuTemperature float64 `json:"cpu_temperature"`
}

// AreaCoverageInfo holds per-area coverage data returned by the ROS service.
type AreaCoverageInfo struct {
	AreaIndex       uint32  `json:"area_index"`
	CoveragePercent float32 `json:"coverage_percent"`
	TotalCells      uint32  `json:"total_cells"`
	MowedCells      uint32  `json:"mowed_cells"`
	ObstacleCells   uint32  `json:"obstacle_cells"`
	StripsRemaining uint32  `json:"strips_remaining"`
}

// CrossChecks holds smart diagnostic cross-checks.
type CrossChecks struct {
	DockPose      DockPoseCheck `json:"dock_pose"`
	Warnings      []string      `json:"warnings"`
	OverallStatus string        `json:"overall_status"` // "ok", "warn", "error"
}

// DockPoseCheck compares configured dock pose with known values.
type DockPoseCheck struct {
	ConfiguredX   float64 `json:"configured_x"`
	ConfiguredY   float64 `json:"configured_y"`
	ConfiguredYaw float64 `json:"configured_yaw"`
	DatumLat      float64 `json:"datum_lat"`
	DatumLon      float64 `json:"datum_lon"`
	HasConfig     bool    `json:"has_config"`
}

// SlamInfo holds SLAM map file metadata (RTAB-Map database).
type SlamInfo struct {
	MapFileExists bool   `json:"map_file_exists"`
	DatabaseSize  int64  `json:"database_size_bytes"`
	LastModified  string `json:"last_modified"`
	MapPath       string `json:"map_path"`
}

// MowingSession represents a single mowing session stored in the DB.
type MowingSession struct {
	ID              string  `json:"id"`
	StartTime       string  `json:"start_time"`
	EndTime         string  `json:"end_time"`
	DurationSec     float64 `json:"duration_sec"`
	AreaIndex       int     `json:"area_index"`
	CoveragePercent float32 `json:"coverage_percent"`
	StripsCompleted uint32  `json:"strips_completed"`
	StripsSkipped   uint32  `json:"strips_skipped"`
	DistanceMeters  float64 `json:"distance_meters"`
	Status          string  `json:"status"` // "completed", "aborted", "error"
	Errors          []string `json:"errors"`
}

// MowingSessionList is the response for GET /diagnostics/sessions.
type MowingSessionList struct {
	Sessions []MowingSession `json:"sessions"`
	Total    int             `json:"total"`
}

// ---------------------------------------------------------------------------
// Route registration
// ---------------------------------------------------------------------------

// DiagnosticsRoutes registers all diagnostics endpoints.
func DiagnosticsRoutes(r *gin.RouterGroup, dockerProvider types.IDockerProvider, rosProvider types.IRosProvider, dbProvider types.IDBProvider) {
	group := r.Group("/diagnostics")
	group.GET("/snapshot", getDiagnosticsSnapshot(dockerProvider, rosProvider, dbProvider))

	// RTAB-Map tools
	group.GET("/slam/info", getSlamInfo())
	group.POST("/slam/save", postSlamSave(rosProvider))
	group.POST("/slam/delete", postSlamDelete(rosProvider))
	group.POST("/slam/mode/mapping", postSlamModeMapping(rosProvider))
	group.POST("/slam/mode/localization", postSlamModeLocalization(rosProvider))

	// Mowing sessions
	group.GET("/sessions", getSessions(dbProvider))
	group.POST("/sessions", postSession(dbProvider))
	group.GET("/sessions/stats", getSessionStats(dbProvider))
}

// ---------------------------------------------------------------------------
// GET /diagnostics/snapshot
// ---------------------------------------------------------------------------

func getDiagnosticsSnapshot(dockerProvider types.IDockerProvider, rosProvider types.IRosProvider, dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 15*time.Second)
		defer cancel()

		snapshot := DiagnosticsSnapshot{
			Timestamp:  time.Now().UTC().Format(time.RFC3339),
			Containers: []ContainerHealth{},
			Coverage:   []AreaCoverageInfo{},
		}

		// --- Containers ---
		if containers, err := dockerProvider.ContainerList(ctx); err == nil {
			for _, ct := range containers {
				name := ""
				if len(ct.Names) > 0 {
					name = strings.TrimPrefix(ct.Names[0], "/")
				}
				startedAt := time.Unix(ct.Created, 0).UTC().Format(time.RFC3339)
				snapshot.Containers = append(snapshot.Containers, ContainerHealth{
					Name:      name,
					State:     ct.State,
					Status:    ct.Status,
					StartedAt: startedAt,
				})
			}
		}

		// --- System ---
		if data, err := os.ReadFile("/sys/class/thermal/thermal_zone0/temp"); err == nil {
			tempStr := strings.TrimSpace(string(data))
			if tempMilliC, err := strconv.ParseFloat(tempStr, 64); err == nil {
				snapshot.System.CpuTemperature = tempMilliC / 1000.0
			}
		}

		// --- Coverage (areas 0..19) ---
		for i := uint32(0); i < 20; i++ {
			req := mowgli.GetCoverageStatusReq{AreaIndex: i}
			var res mowgli.GetCoverageStatusRes
			if err := rosProvider.CallService(ctx, "/map_server_node/get_coverage_status", &req, &res, "mowgli_interfaces/srv/GetCoverageStatus"); err != nil {
				break
			}
			if !res.Success {
				break
			}
			snapshot.Coverage = append(snapshot.Coverage, AreaCoverageInfo{
				AreaIndex:       i,
				CoveragePercent: res.CoveragePercent,
				TotalCells:      res.TotalCells,
				MowedCells:      res.MowedCells,
				ObstacleCells:   res.ObstacleCells,
				StripsRemaining: res.StripsRemaining,
			})
		}

		// --- Cross-checks ---
		snapshot.CrossChecks = buildCrossChecks(dbProvider)

		// --- SLAM info (RTAB-Map database) ---
		snapshot.SlamInfo = readSlamInfo("/ros2_ws/maps/rtabmap.db")

		c.JSON(http.StatusOK, snapshot)
	}
}

// buildCrossChecks reads the robot config and computes diagnostic checks.
func buildCrossChecks(dbProvider types.IDBProvider) CrossChecks {
	checks := CrossChecks{
		Warnings:      []string{},
		OverallStatus: "ok",
	}

	// Read dock pose from mowgli_robot.yaml
	configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		checks.Warnings = append(checks.Warnings, "Cannot read config path: "+err.Error())
		checks.OverallStatus = "warn"
		return checks
	}

	data, err := os.ReadFile(string(configFilePath))
	if err != nil {
		checks.Warnings = append(checks.Warnings, "Cannot read robot config: "+err.Error())
		checks.OverallStatus = "warn"
		return checks
	}

	// Parse the nested YAML (ROS2 format: node_name.ros__parameters.key)
	var yamlData map[string]interface{}
	if err := yaml.Unmarshal(data, &yamlData); err != nil {
		checks.Warnings = append(checks.Warnings, "Cannot parse robot config: "+err.Error())
		checks.OverallStatus = "warn"
		return checks
	}

	// Extract dock pose values from nested YAML
	dockX := extractYAMLFloat(yamlData, "dock_pose_x")
	dockY := extractYAMLFloat(yamlData, "dock_pose_y")
	dockYaw := extractYAMLFloat(yamlData, "dock_pose_yaw")
	datumLat := extractYAMLFloat(yamlData, "datum_lat")
	datumLon := extractYAMLFloat(yamlData, "datum_lon")

	checks.DockPose = DockPoseCheck{
		ConfiguredX:   dockX,
		ConfiguredY:   dockY,
		ConfiguredYaw: dockYaw,
		DatumLat:      datumLat,
		DatumLon:      datumLon,
		HasConfig:     true,
	}

	// Validate config
	if datumLat == 0 && datumLon == 0 {
		checks.Warnings = append(checks.Warnings, "GPS datum not configured (lat=0, lon=0)")
		checks.OverallStatus = "warn"
	}
	if dockYaw == 0 {
		checks.Warnings = append(checks.Warnings, "Dock heading not configured (yaw=0)")
		checks.OverallStatus = "warn"
	}

	return checks
}

// extractYAMLFloat searches recursively for a key in nested YAML.
func extractYAMLFloat(data map[string]interface{}, key string) float64 {
	for k, v := range data {
		if k == key {
			switch val := v.(type) {
			case float64:
				return val
			case int:
				return float64(val)
			}
		}
		if nested, ok := v.(map[string]interface{}); ok {
			if result := extractYAMLFloat(nested, key); result != 0 {
				return result
			}
		}
	}
	return 0
}

// readSlamInfo checks the RTAB-Map database file on disk.
func readSlamInfo(dbPath string) SlamInfo {
	info := SlamInfo{MapPath: dbPath}

	if stat, err := os.Stat(dbPath); err == nil {
		info.MapFileExists = true
		info.DatabaseSize = stat.Size()
		info.LastModified = stat.ModTime().UTC().Format(time.RFC3339)
	}

	return info
}

// ---------------------------------------------------------------------------
// SLAM Map Tools
// ---------------------------------------------------------------------------

// getSlamInfo returns SLAM map file metadata.
func getSlamInfo() gin.HandlerFunc {
	return func(c *gin.Context) {
		info := readSlamInfo("/ros2_ws/maps/rtabmap.db")
		c.JSON(http.StatusOK, info)
	}
}

// postSlamSave triggers an RTAB-Map database backup via the /rtabmap/backup service.
// RTAB-Map persists the live DB automatically on shutdown; this creates a copy to
// /ros2_ws/maps/rtabmap.db.back for manual checkpointing.
func postSlamSave(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 30*time.Second)
		defer cancel()

		// RTAB-Map backup service (std_srvs/srv/Empty — no fields).
		req := struct{}{}
		var res struct{}
		if err := rosProvider.CallService(ctx, "/rtabmap/backup", &req, &res, "std_srvs/srv/Empty"); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to backup RTAB-Map database: " + err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{"ok": true, "message": "RTAB-Map database backed up"})
	}
}

// postSlamModeMapping switches RTAB-Map to mapping mode (adds new nodes to graph).
func postSlamModeMapping(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()

		req := struct{}{}
		var res struct{}
		if err := rosProvider.CallService(ctx, "/rtabmap/set_mode_mapping", &req, &res, "std_srvs/srv/Empty"); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to switch to mapping mode: " + err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"ok": true, "mode": "mapping"})
	}
}

// postSlamModeLocalization switches RTAB-Map to localization mode (read-only map).
func postSlamModeLocalization(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 10*time.Second)
		defer cancel()

		req := struct{}{}
		var res struct{}
		if err := rosProvider.CallService(ctx, "/rtabmap/set_mode_localization", &req, &res, "std_srvs/srv/Empty"); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to switch to localization mode: " + err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"ok": true, "mode": "localization"})
	}
}

// postSlamDelete deletes the RTAB-Map database file. Requires ROS2 container restart.
func postSlamDelete(rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		dbPath := "/ros2_ws/maps/rtabmap.db"
		deleted := []string{}
		errors := []string{}

		// Remove the DB and its typical sidecar files (journal/backup/wal).
		for _, path := range []string{dbPath, dbPath + ".back", dbPath + "-journal", dbPath + "-wal", dbPath + "-shm"} {
			if err := os.Remove(path); err == nil {
				deleted = append(deleted, path)
			} else if !os.IsNotExist(err) {
				errors = append(errors, fmt.Sprintf("Failed to delete %s: %v", path, err))
			}
		}

		if len(errors) > 0 {
			c.JSON(http.StatusInternalServerError, gin.H{
				"ok":      false,
				"deleted": deleted,
				"errors":  errors,
				"message": "Some files could not be deleted. Restart ROS2 to start fresh mapping.",
			})
			return
		}

		c.JSON(http.StatusOK, gin.H{
			"ok":      true,
			"deleted": deleted,
			"message": "SLAM map deleted. Restart ROS2 container to start fresh mapping.",
		})
	}
}

// ---------------------------------------------------------------------------
// Mowing Sessions
// ---------------------------------------------------------------------------

const sessionsDBKey = "mowing.sessions"

// getSessions returns all stored mowing sessions.
func getSessions(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		sessions := loadSessions(dbProvider)
		c.JSON(http.StatusOK, MowingSessionList{
			Sessions: sessions,
			Total:    len(sessions),
		})
	}
}

// postSession stores a new mowing session (appends to the list).
func postSession(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var session MowingSession
		if err := c.BindJSON(&session); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "Invalid session data: " + err.Error()})
			return
		}

		if session.ID == "" {
			session.ID = fmt.Sprintf("%d", time.Now().UnixMilli())
		}

		sessions := loadSessions(dbProvider)
		sessions = append(sessions, session)

		// Keep last 500 sessions max
		if len(sessions) > 500 {
			sessions = sessions[len(sessions)-500:]
		}

		data, err := json.Marshal(sessions)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to marshal sessions"})
			return
		}

		if err := dbProvider.Set(sessionsDBKey, data); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: "Failed to store session: " + err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{"ok": true, "id": session.ID})
	}
}

// getSessionStats returns aggregate statistics across all sessions.
func getSessionStats(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		sessions := loadSessions(dbProvider)

		stats := gin.H{
			"total_sessions":     len(sessions),
			"total_duration_sec": 0.0,
			"total_distance_m":   0.0,
			"total_strips":       0,
			"completed":          0,
			"aborted":            0,
			"errors":             0,
			"avg_coverage_pct":   0.0,
		}

		if len(sessions) == 0 {
			c.JSON(http.StatusOK, stats)
			return
		}

		var totalDur, totalDist float64
		var totalStrips, completed, aborted, errCount int
		var totalCoverage float64

		for _, s := range sessions {
			totalDur += s.DurationSec
			totalDist += s.DistanceMeters
			totalStrips += int(s.StripsCompleted)
			totalCoverage += float64(s.CoveragePercent)
			switch s.Status {
			case "completed":
				completed++
			case "aborted":
				aborted++
			default:
				errCount++
			}
		}

		stats["total_duration_sec"] = math.Round(totalDur*10) / 10
		stats["total_distance_m"] = math.Round(totalDist*10) / 10
		stats["total_strips"] = totalStrips
		stats["completed"] = completed
		stats["aborted"] = aborted
		stats["errors"] = errCount
		stats["avg_coverage_pct"] = math.Round(totalCoverage/float64(len(sessions))*10) / 10

		c.JSON(http.StatusOK, stats)
	}
}

// loadSessions reads the sessions array from the DB.
func loadSessions(dbProvider types.IDBProvider) []MowingSession {
	sessions := []MowingSession{}
	data, err := dbProvider.Get(sessionsDBKey)
	if err != nil {
		return sessions
	}
	_ = json.Unmarshal(data, &sessions)
	return sessions
}
