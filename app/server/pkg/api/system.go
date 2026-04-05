package api

import (
	"net/http"
	"os"
	"os/exec"
	"strconv"
	"strings"

	"github.com/gin-gonic/gin"
)

type SystemInfo struct {
	CPUTemperature *float64 `json:"cpuTemperature,omitempty"`
}

// SystemRoutes registers system info endpoints
func SystemRoutes(r *gin.RouterGroup) {
	group := r.Group("/system")
	group.GET("/info", getSystemInfo)
	group.POST("/reboot", postSystemReboot)
	group.POST("/shutdown", postSystemShutdown)
}

// getSystemInfo returns system information (CPU temperature, etc.)
//
// @Summary get system info
// @Description get system info such as CPU temperature
// @Tags system
// @Produce json
// @Success 200 {object} SystemInfo
// @Router /system/info [get]
func getSystemInfo(c *gin.Context) {
	info := SystemInfo{}

	// Read CPU temperature from thermal zone (Linux/Raspberry Pi)
	if data, err := os.ReadFile("/sys/class/thermal/thermal_zone0/temp"); err == nil {
		tempStr := strings.TrimSpace(string(data))
		if tempMilliC, err := strconv.ParseFloat(tempStr, 64); err == nil {
			temp := tempMilliC / 1000.0
			info.CPUTemperature = &temp
		}
	}

	c.JSON(http.StatusOK, info)
}

// postSystemReboot reboots the host system
//
// @Summary reboot the system
// @Description reboots the Raspberry Pi
// @Tags system
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /system/reboot [post]
func postSystemReboot(c *gin.Context) {
	if err := exec.Command("sudo", "reboot").Start(); err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, OkResponse{})
}

// postSystemShutdown shuts down the host system
//
// @Summary shutdown the system
// @Description shuts down the Raspberry Pi
// @Tags system
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /system/shutdown [post]
func postSystemShutdown(c *gin.Context) {
	if err := exec.Command("sudo", "shutdown", "-h", "now").Start(); err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusOK, OkResponse{})
}
