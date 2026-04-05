package types

type IDBProvider interface {
	// Set sets the value for the given key.
	Set(key string, value []byte) error

	// Get returns the value for the given key.
	Get(key string) ([]byte, error)

	// Delete deletes the value for the given key.
	Delete(key string) error

	// KeysWithSuffix returns a list keys with the give suffix.
	KeysWithSuffix(suffix string) ([]string, error)
}
