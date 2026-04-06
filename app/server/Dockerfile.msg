FROM golang as builder

RUN go install github.com/bluenviron/goroslib/v2/cmd/msg-import@latest
RUN go install github.com/bluenviron/goroslib/v2/cmd/srv-import@latest

FROM builder as msg

COPY ./generate_go_msgs.sh /generate_go_msgs.sh

RUN chmod +x /generate_go_msgs.sh
RUN mkdir -p /app

ENV HOME /app
WORKDIR /app

CMD ["/generate_go_msgs.sh"]
