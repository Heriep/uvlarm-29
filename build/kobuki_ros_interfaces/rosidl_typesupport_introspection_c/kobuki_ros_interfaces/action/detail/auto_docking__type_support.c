// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from kobuki_ros_interfaces:action/AutoDocking.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
#include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
#include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_Goal__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_Goal, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_Goal",  // message name
  1,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_Goal),
  kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_Goal__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_Goal__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_Goal__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_Goal)() {
  if (!kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_Goal__rosidl_typesupport_introspection_c__AutoDocking_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `text`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_Result__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_member_array[1] = {
  {
    "text",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_Result, text),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_Result",  // message name
  1,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_Result),
  kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_Result__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_Result__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_Result__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_Result)() {
  if (!kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_Result__rosidl_typesupport_introspection_c__AutoDocking_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `state`
// Member `text`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_Feedback__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_member_array[2] = {
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_Feedback, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "text",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_Feedback, text),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_Feedback",  // message name
  2,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_Feedback),
  kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_Feedback__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_Feedback__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_Feedback__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_Feedback)() {
  if (!kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_Feedback__rosidl_typesupport_introspection_c__AutoDocking_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "kobuki_ros_interfaces/action/auto_docking.h"
// Member `goal`
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request),
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Request)() {
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_Goal)();
  if (!kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response),
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Response)() {
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "kobuki_ros_interfaces/action/auto_docking.h"
// Member `request`
// Member `response`
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__fini(message_memory);
}

size_t kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_SendGoal_Event__request(
  const void * untyped_member)
{
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_SendGoal_Event__request(
  const void * untyped_member, size_t index)
{
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_SendGoal_Event__request(
  void * untyped_member, size_t index)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_SendGoal_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request * item =
    ((const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request *)
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_SendGoal_Event__request(untyped_member, index));
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request * value =
    (kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request *)(untyped_value);
  *value = *item;
}

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_SendGoal_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request * item =
    ((kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request *)
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_SendGoal_Event__request(untyped_member, index));
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request * value =
    (const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request *)(untyped_value);
  *item = *value;
}

bool kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_SendGoal_Event__request(
  void * untyped_member, size_t size)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence *)(untyped_member);
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence__fini(member);
  return kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__Sequence__init(member, size);
}

size_t kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_SendGoal_Event__response(
  const void * untyped_member)
{
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_SendGoal_Event__response(
  const void * untyped_member, size_t index)
{
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_SendGoal_Event__response(
  void * untyped_member, size_t index)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_SendGoal_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response * item =
    ((const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response *)
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_SendGoal_Event__response(untyped_member, index));
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response * value =
    (kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response *)(untyped_value);
  *value = *item;
}

void kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_SendGoal_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response * item =
    ((kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response *)
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_SendGoal_Event__response(untyped_member, index));
  const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response * value =
    (const kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response *)(untyped_value);
  *item = *value;
}

bool kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_SendGoal_Event__response(
  void * untyped_member, size_t size)
{
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence *)(untyped_member);
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence__fini(member);
  return kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event, request),  // bytes offset in struct
    NULL,  // default value
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_SendGoal_Event__request,  // size() function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_SendGoal_Event__request,  // get_const(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_SendGoal_Event__request,  // get(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_SendGoal_Event__request,  // fetch(index, &value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_SendGoal_Event__request,  // assign(index, value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_SendGoal_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event, response),  // bytes offset in struct
    NULL,  // default value
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_SendGoal_Event__response,  // size() function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_SendGoal_Event__response,  // get_const(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_SendGoal_Event__response,  // get(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_SendGoal_Event__response,  // fetch(index, &value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_SendGoal_Event__response,  // assign(index, value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_SendGoal_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_SendGoal_Event",  // message name
  3,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event),
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Event)() {
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Request)();
  kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Response)();
  if (!kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_service_members = {
  "kobuki_ros_interfaces__action",  // service namespace
  "AutoDocking_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_type_support_handle,
  NULL,  // response message
  // kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_type_support_handle
  NULL  // event_message
  // kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_type_support_handle
};


static rosidl_service_type_support_t kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_service_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_service_members,
  get_service_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Request_message_type_support_handle,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Response_message_type_support_handle,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    kobuki_ros_interfaces,
    action,
    AutoDocking_SendGoal
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    kobuki_ros_interfaces,
    action,
    AutoDocking_SendGoal
  ),
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal__get_type_description_sources,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Response)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Event)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal)() {
  if (!kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_service_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Event)()->data;
  }

  return &kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Request),
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Request)() {
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "kobuki_ros_interfaces/action/auto_docking.h"
// Member `result`
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Response),
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Response)() {
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_Result)();
  if (!kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/service_event_info.h"
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "kobuki_ros_interfaces/action/auto_docking.h"
// Member `request`
// Member `response`
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__fini(message_memory);
}

size_t kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_GetResult_Event__request(
  const void * untyped_member)
{
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_GetResult_Event__request(
  const void * untyped_member, size_t index)
{
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_GetResult_Event__request(
  void * untyped_member, size_t index)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_GetResult_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request * item =
    ((const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request *)
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_GetResult_Event__request(untyped_member, index));
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request * value =
    (kobuki_ros_interfaces__action__AutoDocking_GetResult_Request *)(untyped_value);
  *value = *item;
}

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_GetResult_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request * item =
    ((kobuki_ros_interfaces__action__AutoDocking_GetResult_Request *)
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_GetResult_Event__request(untyped_member, index));
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request * value =
    (const kobuki_ros_interfaces__action__AutoDocking_GetResult_Request *)(untyped_value);
  *item = *value;
}

bool kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_GetResult_Event__request(
  void * untyped_member, size_t size)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence *)(untyped_member);
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence__fini(member);
  return kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__Sequence__init(member, size);
}

size_t kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_GetResult_Event__response(
  const void * untyped_member)
{
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_GetResult_Event__response(
  const void * untyped_member, size_t index)
{
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence * member =
    (const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_GetResult_Event__response(
  void * untyped_member, size_t index)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_GetResult_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response * item =
    ((const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response *)
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_GetResult_Event__response(untyped_member, index));
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response * value =
    (kobuki_ros_interfaces__action__AutoDocking_GetResult_Response *)(untyped_value);
  *value = *item;
}

void kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_GetResult_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response * item =
    ((kobuki_ros_interfaces__action__AutoDocking_GetResult_Response *)
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_GetResult_Event__response(untyped_member, index));
  const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response * value =
    (const kobuki_ros_interfaces__action__AutoDocking_GetResult_Response *)(untyped_value);
  *item = *value;
}

bool kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_GetResult_Event__response(
  void * untyped_member, size_t size)
{
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence * member =
    (kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence *)(untyped_member);
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence__fini(member);
  return kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Event, request),  // bytes offset in struct
    NULL,  // default value
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_GetResult_Event__request,  // size() function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_GetResult_Event__request,  // get_const(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_GetResult_Event__request,  // get(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_GetResult_Event__request,  // fetch(index, &value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_GetResult_Event__request,  // assign(index, value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_GetResult_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Event, response),  // bytes offset in struct
    NULL,  // default value
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__size_function__AutoDocking_GetResult_Event__response,  // size() function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__AutoDocking_GetResult_Event__response,  // get_const(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__get_function__AutoDocking_GetResult_Event__response,  // get(index) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__AutoDocking_GetResult_Event__response,  // fetch(index, &value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__AutoDocking_GetResult_Event__response,  // assign(index, value) function pointer
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__AutoDocking_GetResult_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_GetResult_Event",  // message name
  3,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_GetResult_Event),
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Event)() {
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Request)();
  kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Response)();
  if (!kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_service_members = {
  "kobuki_ros_interfaces__action",  // service namespace
  "AutoDocking_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_type_support_handle,
  NULL,  // response message
  // kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_type_support_handle
  NULL  // event_message
  // kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_type_support_handle
};


static rosidl_service_type_support_t kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_service_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_service_members,
  get_service_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Request_message_type_support_handle,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Response_message_type_support_handle,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__rosidl_typesupport_introspection_c__AutoDocking_GetResult_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    kobuki_ros_interfaces,
    action,
    AutoDocking_GetResult
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    kobuki_ros_interfaces,
    action,
    AutoDocking_GetResult
  ),
  &kobuki_ros_interfaces__action__AutoDocking_GetResult__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult__get_type_description_sources,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Response)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Event)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult)() {
  if (!kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_service_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_GetResult_Event)()->data;
  }

  return &kobuki_ros_interfaces__action__detail__auto_docking__rosidl_typesupport_introspection_c__AutoDocking_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"
// already included above
// #include "kobuki_ros_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "kobuki_ros_interfaces/action/auto_docking.h"
// Member `feedback`
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__init(message_memory);
}

void kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_fini_function(void * message_memory)
{
  kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_members = {
  "kobuki_ros_interfaces__action",  // message namespace
  "AutoDocking_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage),
  kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_member_array,  // message members
  kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_type_support_handle = {
  0,
  &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kobuki_ros_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_FeedbackMessage)() {
  kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kobuki_ros_interfaces, action, AutoDocking_Feedback)();
  if (!kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__rosidl_typesupport_introspection_c__AutoDocking_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
