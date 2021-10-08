// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/tools/map_datachecker/proto/collection_error_code.proto

#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace apollo {
namespace hdmap {
}  // namespace hdmap
}  // namespace apollo
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nCmodules/map/tools/map_datachecker/prot"
  "o/collection_error_code.proto\022\014apollo.hd"
  "map*\253\005\n\tErrorCode\022\013\n\007SUCCESS\020\000\022\t\n\005ERROR\020"
  "\001\022\021\n\rERROR_REQUEST\020\002\022\035\n\031ERROR_SERVICE_NO"
  "_RESPONSE\020\003\022\030\n\024ERROR_REPEATED_START\020\004\022\034\n"
  "\030ERROR_CHECK_BEFORE_START\020\005\022\025\n\021ERROR_GPS"
  "BIN_LACK\020e\022\030\n\024ERROR_DISKINFO_ERROR\020f\022\026\n\022"
  "ERROR_DISK_UNMOUNT\020g\022\024\n\020ERROR_SPEED_LACK"
  "\020i\022\031\n\025WARNING_ODOMETER_LACK\020j\022\031\n\025ERROR_R"
  "TKSTATUS_EMPTY\020k\022\036\n\031ERROR_MAPGRPC_NOT_CO"
  "NNECT\020\311\001\022\031\n\024WARNING_NOT_STRAIGHT\020\324\001\022\036\n\031W"
  "ARNING_PROGRESS_ROLLBACK\020\325\001\022\032\n\025ERROR_NOT"
  "_EIGHT_ROUTE\020\335\001\022$\n\037ERROR_CHANNEL_VERIFY_"
  "TOPIC_LACK\020\347\001\022(\n#ERROR_CHANNEL_VERIFY_RA"
  "TES_ABNORMAL\020\350\001\022\036\n\031ERROR_VERIFY_NO_RECOR"
  "DERS\020\351\001\022\034\n\027ERROR_LOOPS_NOT_REACHED\020\352\001\022\034\n"
  "\027ERROR_VERIFY_NO_GNSSPOS\020\353\001\022\025\n\020ERROR_NOT"
  "_STATIC\020\361\001\022\033\n\026ERROR_GNSS_SIGNAL_FAIL\020\362\001\022"
  "\027\n\022SUCCESS_TASK_EMPTY\020\255\002\022\027\n\022SUCCESS_TASK"
  "_STOCK\020\256\002"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto = {
  false, false, 769, descriptor_table_protodef_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto, "modules/map/tools/map_datachecker/proto/collection_error_code.proto", 
  &descriptor_table_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto_once, nullptr, 0, 0,
  schemas, file_default_instances, TableStruct_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto::offsets,
  nullptr, file_level_enum_descriptors_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto, file_level_service_descriptors_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto_getter() {
  return &descriptor_table_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto(&descriptor_table_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto);
namespace apollo {
namespace hdmap {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ErrorCode_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto);
  return file_level_enum_descriptors_modules_2fmap_2ftools_2fmap_5fdatachecker_2fproto_2fcollection_5ferror_5fcode_2eproto[0];
}
bool ErrorCode_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 101:
    case 102:
    case 103:
    case 105:
    case 106:
    case 107:
    case 201:
    case 212:
    case 213:
    case 221:
    case 231:
    case 232:
    case 233:
    case 234:
    case 235:
    case 241:
    case 242:
    case 301:
    case 302:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
