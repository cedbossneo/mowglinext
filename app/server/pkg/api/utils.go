package api

import (
	"encoding/json"
	"github.com/mitchellh/mapstructure"
	"io"
	"strings"
	"unicode"
)

func snakeToCamel(in string) string {
	tmp := []rune(in)
	tmp[0] = unicode.ToUpper(tmp[0])
	for i := 0; i < len(tmp); i++ {
		if tmp[i] == '_' {
			tmp[i+1] = unicode.ToUpper(tmp[i+1])
			tmp = append(tmp[:i], tmp[i+1:]...)
			i--
		}
	}
	return string(tmp)
}

func unmarshalROSMessage[T any](reader io.ReadCloser, out T) error {
	var m map[string]interface{}
	all, err := io.ReadAll(reader)
	if err != nil {
		return err
	}
	err = json.Unmarshal(all, &m)
	if err != nil {
		return err
	}
	decoder, err := mapstructure.NewDecoder(&mapstructure.DecoderConfig{
		Result: out,
		MatchName: func(mapKey, fieldName string) bool {
			return strings.ToLower(fieldName) == strings.ToLower(mapKey)
		},
	})
	err = decoder.Decode(m)
	if err != nil {
		return err
	}
	return nil
}
