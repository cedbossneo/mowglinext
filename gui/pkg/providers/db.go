package providers

import (
	"errors"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"strings"
	"time"

	"git.mills.io/prologic/bitcask"
	"golang.org/x/xerrors"
)

// quarantinePrefix names the backup directory that holds a store which was
// too corrupted to recover. It is skipped when quarantining so repeated
// corruptions don't nest previous backups inside each other.
const quarantinePrefix = "corrupt-"

type DBProvider struct {
	db *bitcask.Bitcask
}

var EnvFallbacks = map[string]string{
	"system.api.addr":         "API_ADDR",
	"system.api.webDirectory": "WEB_DIR",
	"system.map.enabled":      "MAP_TILE_ENABLED",
	"system.map.tileServer":   "MAP_TILE_SERVER",
	"system.map.tileUri":      "MAP_TILE_URI",
	"system.homekit.enabled":  "HOMEKIT_ENABLED",
	"system.mqtt.enabled":     "MQTT_ENABLED",
	"system.mqtt.prefix":      "MQTT_PREFIX",
	"system.mqtt.host":        "MQTT_HOST",
	"system.mower.configFile":     "MOWER_CONFIG_FILE",
	"system.mower.yamlConfigFile": "MOWER_YAML_CONFIG_FILE",
	"system.mower.runtimeEnvFile": "MOWER_RUNTIME_ENV_FILE",
	"system.ros.masterUri":     "ROS_MASTER_URI",
	"system.ros.nodeName":      "ROS_NODE_NAME",
	"system.ros.nodeHost":      "ROS_NODE_HOST",
	"system.ros.foxgloveUrl":   "FOXGLOVE_URL",
	"system.homekit.pincode":  "HOMEKIT_PINCODE",
}
var Defaults = map[string]string{
	"system.api.addr":         ":4006",
	"system.api.webDirectory": "/app/web",
	"system.map.enabled":      "false",
	"system.map.tileServer":   "http://localhost:5000",
	"system.map.tileUri":      "/tiles/vt/lyrs=s,h&x={x}&y={y}&z={z}",
	"system.homekit.enabled":  "false",
	"system.homekit.pincode":  "00102003",
	"system.mqtt.enabled":     "false",
	"system.mqtt.host":        ":1883",
	"system.mqtt.prefix":      "/gui",
	"system.mower.configFile":     "/config/mower_config.sh",
	"system.mower.yamlConfigFile": "/config/mowgli_robot.yaml",
	"system.mower.runtimeEnvFile": "/runtime_config/.env",
	"system.ros.masterUri":    "http://localhost:11311",
	"system.ros.nodeName":     "mowglinext",
	"system.ros.nodeHost":     "localhost",
}

func (d *DBProvider) Set(key string, value []byte) error {
	return d.db.Put([]byte(key), value)
}

func (d *DBProvider) Get(key string) ([]byte, error) {
	value, err := d.db.Get([]byte(key))
	if err != nil || value == nil || len(value) == 0 {
		if !errors.Is(err, bitcask.ErrKeyNotFound) {
			return nil, err
		}
		if EnvFallbacks[key] != "" && os.Getenv(EnvFallbacks[key]) != "" {
			return []byte(os.Getenv(EnvFallbacks[key])), nil
		}
		if Defaults[key] != "" {
			return []byte(Defaults[key]), nil
		}
		return nil, xerrors.Errorf("config key %s not found", key)
	}
	return value, nil
}

func (d *DBProvider) Delete(key string) error {
	return d.db.Delete([]byte(key))
}

func (d *DBProvider) KeysWithSuffix(suffix string) ([]string, error) {
	var keys []string
	err := d.db.Scan([]byte(suffix), func(key []byte) error {
		keys = append(keys, string(key))
		return nil
	})
	if err != nil {
		return nil, err
	}
	return keys, nil
}

func (d *DBProvider) GetWithEnvFallback(key string, env string, def string) string {
	value, err := d.Get(key)
	if err != nil || value == nil || len(value) == 0 {
		if os.Getenv(env) == "" {
			return def
		} else {
			return os.Getenv(env)
		}
	}
	return string(value)
}

func NewDBProvider() *DBProvider {
	db, err := openResilient(os.Getenv("DB_PATH"))
	if err != nil {
		panic(err)
	}
	return &DBProvider{db: db}
}

// openResilient opens the bitcask store, tolerating a store left corrupted by
// an unclean shutdown (power loss mid-write is a normal occurrence for a
// robot). It first enables bitcask's auto-recovery, which truncates the
// corrupted tail of the last datafile and preserves every intact key. If the
// store is still unopenable (e.g. a corrupted meta.json/config.json that
// auto-recovery does not cover), it moves the whole store aside and recreates
// a fresh one — the operator's proven manual workaround — so the GUI stays
// reachable instead of crash-looping. The bitcask store only holds GUI-side
// state (onboarding flag, session history, schedule, map offset); robot
// calibration and maps live elsewhere, so recreating it loses nothing critical.
func openResilient(path string) (*bitcask.Bitcask, error) {
	db, err := bitcask.Open(path, bitcask.WithAutoRecovery(true))
	if err == nil {
		return db, nil
	}

	log.Printf("DBProvider: store at %q is unrecoverable (%v); quarantining it and starting fresh", path, err)
	if qErr := quarantineStore(path); qErr != nil {
		return nil, xerrors.Errorf("db store corrupted (%v) and quarantine failed: %w", err, qErr)
	}
	return bitcask.Open(path, bitcask.WithAutoRecovery(true))
}

// quarantineStore moves the contents of the store directory into a timestamped
// backup subdirectory, leaving the directory itself in place (it may be a bind
// mount that cannot be renamed) so a fresh store can be created there. Previous
// quarantine backups are left untouched rather than nested.
func quarantineStore(path string) error {
	entries, err := os.ReadDir(path)
	if err != nil {
		return err
	}

	backupDir := filepath.Join(path, fmt.Sprintf("%s%d", quarantinePrefix, time.Now().Unix()))
	if err := os.MkdirAll(backupDir, 0o755); err != nil {
		return err
	}

	moved := 0
	for _, entry := range entries {
		name := entry.Name()
		if strings.HasPrefix(name, quarantinePrefix) {
			continue
		}
		if err := os.Rename(filepath.Join(path, name), filepath.Join(backupDir, name)); err != nil {
			return xerrors.Errorf("moving %s aside: %w", name, err)
		}
		moved++
	}

	if moved == 0 {
		_ = os.Remove(backupDir)
	}
	return nil
}
