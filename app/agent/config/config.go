package config

import "os"

type Config struct {
	ProxyURL    string
	RobotToken  string
	RobotID     string
	LocalGUIURL string
}

func Load() *Config {
	return &Config{
		ProxyURL:    getEnv("PROXY_URL", ""),
		RobotToken:  getEnv("ROBOT_TOKEN", ""),
		RobotID:     getEnv("ROBOT_ID", ""),
		LocalGUIURL: getEnv("LOCAL_GUI_URL", "http://localhost:80"),
	}
}

func getEnv(key, fallback string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return fallback
}
