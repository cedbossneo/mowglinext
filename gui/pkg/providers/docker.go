package providers

import (
	"context"
	"errors"
	types2 "github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/docker/docker/api/types"
	docker "github.com/docker/docker/client"
	"github.com/sirupsen/logrus"
	"io"
)

type DockerProvider struct {
	client *docker.Client
}

func NewDockerProvider() types2.IDockerProvider {
	client, err := docker.NewClientWithOpts(docker.FromEnv)
	if err != nil {
		logrus.Error(err)
	}
	return &DockerProvider{client: client}
}

func (i *DockerProvider) ContainerList(ctx context.Context) ([]types.Container, error) {
	if i.client == nil {
		return nil, errors.New("docker client is not initialized")
	}
	return i.client.ContainerList(ctx, types.ContainerListOptions{
		All: true,
	})
}

func (i *DockerProvider) ContainerLogs(ctx context.Context, containerID string) (io.ReadCloser, error) {
	if i.client == nil {
		return nil, errors.New("docker client is not initialized")
	}
	return i.client.ContainerLogs(ctx, containerID, types.ContainerLogsOptions{ShowStdout: true, ShowStderr: true, Follow: true, Tail: "100"})
}

func (i *DockerProvider) ContainerStart(ctx context.Context, containerID string) error {
	if i.client == nil {
		return errors.New("docker client is not initialized")
	}
	return i.client.ContainerStart(ctx, containerID, types.ContainerStartOptions{})
}

func (i *DockerProvider) ContainerStop(ctx context.Context, containerID string) error {
	if i.client == nil {
		return errors.New("docker client is not initialized")
	}
	return i.client.ContainerStop(ctx, containerID, nil)
}

func (i *DockerProvider) ContainerRestart(ctx context.Context, containerID string) error {
	return i.client.ContainerRestart(ctx, containerID, nil)
}
