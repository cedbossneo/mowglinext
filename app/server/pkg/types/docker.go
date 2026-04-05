package types

import (
	"context"
	"github.com/docker/docker/api/types"
	"io"
)

type IDockerProvider interface {
	ContainerList(ctx context.Context) ([]types.Container, error)
	ContainerLogs(ctx context.Context, containerID string) (io.ReadCloser, error)
	ContainerStart(ctx context.Context, containerID string) error
	ContainerStop(ctx context.Context, containerID string) error
	ContainerRestart(ctx context.Context, containerID string) error
}
