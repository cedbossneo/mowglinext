package config

import "os"

type Config struct {
	MongoURI       string
	JWTSecret      string
	Port           string
	FCMCredentials string
}

func Load() *Config {
	return &Config{
		MongoURI:       getEnv("MONGO_URI", "mongodb://localhost:27017/mowglinext"),
		JWTSecret:      getEnv("JWT_SECRET", "change-me-in-production"),
		Port:           getEnv("PORT", "8080"),
		FCMCredentials: getEnv("FCM_CREDENTIALS", ""),
	}
}

func getEnv(key, fallback string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return fallback
}
