#include "fms_fleet_server/rabbitmq_client.hpp"

#include <amqp_framing.h>
#include <amqp_tcp_socket.h>

#include <stdexcept>

namespace fms_fleet_server {

namespace {

void check_reply(amqp_rpc_reply_t reply, const std::string& context)
{
  if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
    throw std::runtime_error("RabbitMQ error during " + context);
  }
}

}  // namespace

RabbitMqClient::RabbitMqClient(const std::string& host, int port,
                                const std::string& user, const std::string& password)
{
  conn_ = amqp_new_connection();

  amqp_socket_t* socket = amqp_tcp_socket_new(conn_);
  if (!socket) {
    throw std::runtime_error("RabbitMQ: failed to create TCP socket");
  }
  if (amqp_socket_open(socket, host.c_str(), port) != AMQP_STATUS_OK) {
    throw std::runtime_error("RabbitMQ: failed to connect to " + host + ":" + std::to_string(port));
  }

  check_reply(amqp_login(conn_, "/", 0, 131072, 0, AMQP_SASL_METHOD_PLAIN,
                          user.c_str(), password.c_str()),
              "login");

  amqp_channel_open(conn_, kChannel);
  check_reply(amqp_get_rpc_reply(conn_), "channel.open");
}

RabbitMqClient::~RabbitMqClient()
{
  amqp_channel_close(conn_, kChannel, AMQP_REPLY_SUCCESS);
  amqp_connection_close(conn_, AMQP_REPLY_SUCCESS);
  amqp_destroy_connection(conn_);
}

void RabbitMqClient::setup_topology()
{
  // Dead-letter exchange + queue for rejected/failed tasks.
  amqp_exchange_declare(conn_, kChannel, amqp_cstring_bytes("tasks.dlx"),
                          amqp_cstring_bytes("direct"), 0, 1, 0, 0, amqp_empty_table);
  check_reply(amqp_get_rpc_reply(conn_), "exchange.declare tasks.dlx");

  amqp_queue_declare(conn_, kChannel, amqp_cstring_bytes("tasks.failed"),
                       0, 1, 0, 0, amqp_empty_table);
  check_reply(amqp_get_rpc_reply(conn_), "queue.declare tasks.failed");

  amqp_queue_bind(conn_, kChannel, amqp_cstring_bytes("tasks.failed"),
                    amqp_cstring_bytes("tasks.dlx"), amqp_cstring_bytes("failed"),
                    amqp_empty_table);
  check_reply(amqp_get_rpc_reply(conn_), "queue.bind tasks.failed");

  // Primary task exchange + incoming queue, with DLX routing for rejected messages.
  amqp_exchange_declare(conn_, kChannel, amqp_cstring_bytes("tasks"),
                          amqp_cstring_bytes("direct"), 0, 1, 0, 0, amqp_empty_table);
  check_reply(amqp_get_rpc_reply(conn_), "exchange.declare tasks");

  amqp_table_entry_t dlx_entries[2];
  dlx_entries[0].key = amqp_cstring_bytes("x-dead-letter-exchange");
  dlx_entries[0].value.kind = AMQP_FIELD_KIND_UTF8;
  dlx_entries[0].value.value.bytes = amqp_cstring_bytes("tasks.dlx");
  dlx_entries[1].key = amqp_cstring_bytes("x-dead-letter-routing-key");
  dlx_entries[1].value.kind = AMQP_FIELD_KIND_UTF8;
  dlx_entries[1].value.value.bytes = amqp_cstring_bytes("failed");

  amqp_table_t queue_args;
  queue_args.num_entries = 2;
  queue_args.entries = dlx_entries;

  amqp_queue_declare(conn_, kChannel, amqp_cstring_bytes("tasks.incoming"),
                       0, 1, 0, 0, queue_args);
  check_reply(amqp_get_rpc_reply(conn_), "queue.declare tasks.incoming");

  amqp_queue_bind(conn_, kChannel, amqp_cstring_bytes("tasks.incoming"),
                    amqp_cstring_bytes("tasks"), amqp_cstring_bytes("incoming"),
                    amqp_empty_table);
  check_reply(amqp_get_rpc_reply(conn_), "queue.bind tasks.incoming");

  // Manual ack so we can route unassignable tasks to the DLX via reject().
  amqp_basic_consume(conn_, kChannel, amqp_cstring_bytes("tasks.incoming"),
                       amqp_empty_bytes, 0, 0, 0, amqp_empty_table);
  check_reply(amqp_get_rpc_reply(conn_), "basic.consume tasks.incoming");
}

void RabbitMqClient::publish(const std::string& exchange, const std::string& routing_key,
                              const std::string& body)
{
  amqp_basic_properties_t props;
  props._flags = AMQP_BASIC_CONTENT_TYPE_FLAG | AMQP_BASIC_DELIVERY_MODE_FLAG;
  props.content_type = amqp_cstring_bytes("application/json");
  props.delivery_mode = 2;  // persistent

  int rc = amqp_basic_publish(conn_, kChannel, amqp_cstring_bytes(exchange.c_str()),
                                amqp_cstring_bytes(routing_key.c_str()), 0, 0, &props,
                                amqp_cstring_bytes(body.c_str()));
  if (rc != AMQP_STATUS_OK) {
    throw std::runtime_error("RabbitMQ: basic.publish failed on exchange " + exchange);
  }
}

void RabbitMqClient::publish_failed(const std::string& json_body)
{
  publish("tasks.dlx", "failed", json_body);
}

bool RabbitMqClient::consume_incoming(Message& out, int timeout_sec)
{
  amqp_maybe_release_buffers(conn_);

  struct timeval timeout;
  timeout.tv_sec = timeout_sec;
  timeout.tv_usec = 0;

  amqp_envelope_t envelope;
  amqp_rpc_reply_t reply = amqp_consume_message(conn_, &envelope, &timeout, 0);

  if (reply.reply_type != AMQP_RESPONSE_NORMAL) {
    if (reply.reply_type == AMQP_RESPONSE_LIBRARY_EXCEPTION &&
        reply.library_error == AMQP_STATUS_TIMEOUT) {
      return false;
    }
    throw std::runtime_error("RabbitMQ: consume failed");
  }

  out.delivery_tag = envelope.delivery_tag;
  out.body.assign(static_cast<const char*>(envelope.message.body.bytes),
                    envelope.message.body.len);

  amqp_destroy_envelope(&envelope);
  return true;
}

void RabbitMqClient::ack(uint64_t delivery_tag)
{
  amqp_basic_ack(conn_, kChannel, delivery_tag, 0);
}

void RabbitMqClient::reject(uint64_t delivery_tag, bool requeue)
{
  amqp_basic_reject(conn_, kChannel, delivery_tag, requeue ? 1 : 0);
}

}  // namespace fms_fleet_server
