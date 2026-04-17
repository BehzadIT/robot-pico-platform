#pragma once

#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/transport_types.hpp"

namespace robot::runtime {

/**
 * Shared inter-core state for the staged Pico SDK runtime.
 *
 * The final firmware will place service-side ingress on one core and control
 * ownership on the other. These bounded mailboxes are the only allowed handoff
 * path between those domains.
 */
struct SharedRuntimeState {
  Mailbox<TransportCommand, 8> inbound_commands;
  Mailbox<ControlCompletion, 8> outbound_completions;
};

}  // namespace robot::runtime
