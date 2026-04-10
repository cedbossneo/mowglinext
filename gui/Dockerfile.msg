FROM bash:latest

COPY ./generate_go_msgs.sh /generate_go_msgs.sh
RUN chmod +x /generate_go_msgs.sh

WORKDIR /app

CMD ["/generate_go_msgs.sh"]
