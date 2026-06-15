#pragma once

#include <amqp.h>

#include <cstdint>
#include <string>

namespace fms_fleet_server {

// Thin wrapper around librabbitmq (rabbitmq-c) for the fleet server's task
// broker. Sets up:
//   - exchange "tasks" (direct), queue "tasks.incoming" bound to it with
//     routing key "incoming". The queue is declared with a dead-letter
//     exchange so rejected/expired messages land in "tasks.dlx".
//   - exchange "tasks.dlx" (direct), queue "tasks.failed" bound to it with
//     routing key "failed" — holds tasks that could not be assigned, and is
//     also used directly to publish execution-time failures.
class RabbitMqClient {
public:
  RabbitMqClient(const std::string& host, int port,
                  const std::string& user, const std::string& password);
  ~RabbitMqClient();

  RabbitMqClient(const RabbitMqClient&) = delete;
  RabbitMqClient& operator=(const RabbitMqClient&) = delete;

  // Declares exchanges/queues/bindings described above and starts consuming
  // from "tasks.incoming" (manual ack).
  void setup_topology();

  // Publish a JSON document to the "tasks.dlx" exchange with routing key
  // "failed" (used for execution-time task failures/cancellations).
  void publish_failed(const std::string& json_body);

  struct Message {
    uint64_t delivery_tag;
    std::string body;
  };

  // Blocks up to timeout_sec waiting for the next "tasks.incoming" message.
  // Returns false on timeout (out is left untouched).
  bool consume_incoming(Message& out, int timeout_sec);

  void ack(uint64_t delivery_tag);
  // requeue=false routes the message to the queue's dead-letter exchange.
  void reject(uint64_t delivery_tag, bool requeue);

private:
  void publish(const std::string& exchange, const std::string& routing_key,
               const std::string& body);

  amqp_connection_state_t conn_;
  static constexpr amqp_channel_t kChannel = 1;
};

}  // namespace fms_fleet_server
