// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from kobuki_ros_interfaces:action/AutoDocking.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
#include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_Goal_type_support_ids_t;

static const _AutoDocking_Goal_type_support_ids_t _AutoDocking_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_Goal_type_support_symbol_names_t _AutoDocking_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_Goal)),
  }
};

typedef struct _AutoDocking_Goal_type_support_data_t
{
  void * data[2];
} _AutoDocking_Goal_type_support_data_t;

static _AutoDocking_Goal_type_support_data_t _AutoDocking_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_Goal_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_Goal__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_Goal__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_Goal>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_Goal)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_Result_type_support_ids_t;

static const _AutoDocking_Result_type_support_ids_t _AutoDocking_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_Result_type_support_symbol_names_t _AutoDocking_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_Result)),
  }
};

typedef struct _AutoDocking_Result_type_support_data_t
{
  void * data[2];
} _AutoDocking_Result_type_support_data_t;

static _AutoDocking_Result_type_support_data_t _AutoDocking_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_Result_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_Result_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_Result_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_Result__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_Result__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_Result>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_Result)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_Feedback_type_support_ids_t;

static const _AutoDocking_Feedback_type_support_ids_t _AutoDocking_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_Feedback_type_support_symbol_names_t _AutoDocking_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_Feedback)),
  }
};

typedef struct _AutoDocking_Feedback_type_support_data_t
{
  void * data[2];
} _AutoDocking_Feedback_type_support_data_t;

static _AutoDocking_Feedback_type_support_data_t _AutoDocking_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_Feedback_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_Feedback__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_Feedback__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_Feedback>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_Feedback)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_SendGoal_Request_type_support_ids_t;

static const _AutoDocking_SendGoal_Request_type_support_ids_t _AutoDocking_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_SendGoal_Request_type_support_symbol_names_t _AutoDocking_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Request)),
  }
};

typedef struct _AutoDocking_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _AutoDocking_SendGoal_Request_type_support_data_t;

static _AutoDocking_SendGoal_Request_type_support_data_t _AutoDocking_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_SendGoal_Request_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Request>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Request)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_SendGoal_Response_type_support_ids_t;

static const _AutoDocking_SendGoal_Response_type_support_ids_t _AutoDocking_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_SendGoal_Response_type_support_symbol_names_t _AutoDocking_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Response)),
  }
};

typedef struct _AutoDocking_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _AutoDocking_SendGoal_Response_type_support_data_t;

static _AutoDocking_SendGoal_Response_type_support_data_t _AutoDocking_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_SendGoal_Response_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Response>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Response)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_SendGoal_Event_type_support_ids_t;

static const _AutoDocking_SendGoal_Event_type_support_ids_t _AutoDocking_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_SendGoal_Event_type_support_symbol_names_t _AutoDocking_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Event)),
  }
};

typedef struct _AutoDocking_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _AutoDocking_SendGoal_Event_type_support_data_t;

static _AutoDocking_SendGoal_Event_type_support_data_t _AutoDocking_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_SendGoal_Event_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Event>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal_Event)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_SendGoal_type_support_ids_t;

static const _AutoDocking_SendGoal_type_support_ids_t _AutoDocking_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_SendGoal_type_support_symbol_names_t _AutoDocking_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_SendGoal)),
  }
};

typedef struct _AutoDocking_SendGoal_type_support_data_t
{
  void * data[2];
} _AutoDocking_SendGoal_type_support_data_t;

static _AutoDocking_SendGoal_type_support_data_t _AutoDocking_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_SendGoal_service_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t AutoDocking_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<kobuki_ros_interfaces::action::AutoDocking_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<kobuki_ros_interfaces::action::AutoDocking_SendGoal>,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_SendGoal>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_GetResult_Request_type_support_ids_t;

static const _AutoDocking_GetResult_Request_type_support_ids_t _AutoDocking_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_GetResult_Request_type_support_symbol_names_t _AutoDocking_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Request)),
  }
};

typedef struct _AutoDocking_GetResult_Request_type_support_data_t
{
  void * data[2];
} _AutoDocking_GetResult_Request_type_support_data_t;

static _AutoDocking_GetResult_Request_type_support_data_t _AutoDocking_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_GetResult_Request_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Request>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Request)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_GetResult_Response_type_support_ids_t;

static const _AutoDocking_GetResult_Response_type_support_ids_t _AutoDocking_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_GetResult_Response_type_support_symbol_names_t _AutoDocking_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Response)),
  }
};

typedef struct _AutoDocking_GetResult_Response_type_support_data_t
{
  void * data[2];
} _AutoDocking_GetResult_Response_type_support_data_t;

static _AutoDocking_GetResult_Response_type_support_data_t _AutoDocking_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_GetResult_Response_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Response>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Response)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_GetResult_Event_type_support_ids_t;

static const _AutoDocking_GetResult_Event_type_support_ids_t _AutoDocking_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_GetResult_Event_type_support_symbol_names_t _AutoDocking_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Event)),
  }
};

typedef struct _AutoDocking_GetResult_Event_type_support_data_t
{
  void * data[2];
} _AutoDocking_GetResult_Event_type_support_data_t;

static _AutoDocking_GetResult_Event_type_support_data_t _AutoDocking_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_GetResult_Event_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Event>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult_Event)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_GetResult_type_support_ids_t;

static const _AutoDocking_GetResult_type_support_ids_t _AutoDocking_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_GetResult_type_support_symbol_names_t _AutoDocking_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_GetResult)),
  }
};

typedef struct _AutoDocking_GetResult_type_support_data_t
{
  void * data[2];
} _AutoDocking_GetResult_type_support_data_t;

static _AutoDocking_GetResult_type_support_data_t _AutoDocking_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_GetResult_service_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t AutoDocking_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<kobuki_ros_interfaces::action::AutoDocking_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<kobuki_ros_interfaces::action::AutoDocking_GetResult>,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_GetResult>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__functions.h"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _AutoDocking_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _AutoDocking_FeedbackMessage_type_support_ids_t;

static const _AutoDocking_FeedbackMessage_type_support_ids_t _AutoDocking_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _AutoDocking_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _AutoDocking_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _AutoDocking_FeedbackMessage_type_support_symbol_names_t _AutoDocking_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, kobuki_ros_interfaces, action, AutoDocking_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kobuki_ros_interfaces, action, AutoDocking_FeedbackMessage)),
  }
};

typedef struct _AutoDocking_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _AutoDocking_FeedbackMessage_type_support_data_t;

static _AutoDocking_FeedbackMessage_type_support_data_t _AutoDocking_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _AutoDocking_FeedbackMessage_message_typesupport_map = {
  2,
  "kobuki_ros_interfaces",
  &_AutoDocking_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_AutoDocking_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_AutoDocking_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t AutoDocking_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_AutoDocking_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_FeedbackMessage>()
{
  return &::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, kobuki_ros_interfaces, action, AutoDocking_FeedbackMessage)() {
  return get_message_type_support_handle<kobuki_ros_interfaces::action::AutoDocking_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "kobuki_ros_interfaces/action/detail/auto_docking__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace kobuki_ros_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t AutoDocking_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &kobuki_ros_interfaces__action__AutoDocking__get_type_hash,
  &kobuki_ros_interfaces__action__AutoDocking__get_type_description,
  &kobuki_ros_interfaces__action__AutoDocking__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace kobuki_ros_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<kobuki_ros_interfaces::action::AutoDocking>()
{
  using ::kobuki_ros_interfaces::action::rosidl_typesupport_cpp::AutoDocking_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  AutoDocking_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::kobuki_ros_interfaces::action::AutoDocking::Impl::SendGoalService>();
  AutoDocking_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::kobuki_ros_interfaces::action::AutoDocking::Impl::GetResultService>();
  AutoDocking_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::kobuki_ros_interfaces::action::AutoDocking::Impl::CancelGoalService>();
  AutoDocking_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::kobuki_ros_interfaces::action::AutoDocking::Impl::FeedbackMessage>();
  AutoDocking_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::kobuki_ros_interfaces::action::AutoDocking::Impl::GoalStatusMessage>();
  return &AutoDocking_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp