// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.proto

#include "modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.pb.h"

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
namespace perception {
namespace lidar {
constexpr SceneManagerConfig::SceneManagerConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : service_name_(){}
struct SceneManagerConfigDefaultTypeInternal {
  constexpr SceneManagerConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SceneManagerConfigDefaultTypeInternal() {}
  union {
    SceneManagerConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SceneManagerConfigDefaultTypeInternal _SceneManagerConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SceneManagerConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::SceneManagerConfig, service_name_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::apollo::perception::lidar::SceneManagerConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_SceneManagerConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nKmodules/perception/lidar/lib/scene_man"
  "ager/proto/scene_manager_config.proto\022\027a"
  "pollo.perception.lidar\"*\n\022SceneManagerCo"
  "nfig\022\024\n\014service_name\030\001 \003(\t"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto = {
  false, false, 146, descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto, "modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.proto", 
  &descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto(&descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class SceneManagerConfig::_Internal {
 public:
};

SceneManagerConfig::SceneManagerConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  service_name_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.SceneManagerConfig)
}
SceneManagerConfig::SceneManagerConfig(const SceneManagerConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      service_name_(from.service_name_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.SceneManagerConfig)
}

void SceneManagerConfig::SharedCtor() {
}

SceneManagerConfig::~SceneManagerConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.SceneManagerConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SceneManagerConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void SceneManagerConfig::ArenaDtor(void* object) {
  SceneManagerConfig* _this = reinterpret_cast< SceneManagerConfig* >(object);
  (void)_this;
}
void SceneManagerConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SceneManagerConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SceneManagerConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.SceneManagerConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  service_name_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SceneManagerConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated string service_name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            auto str = _internal_add_service_name();
            ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
            #ifndef NDEBUG
            ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.SceneManagerConfig.service_name");
            #endif  // !NDEBUG
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* SceneManagerConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.SceneManagerConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated string service_name = 1;
  for (int i = 0, n = this->_internal_service_name_size(); i < n; i++) {
    const auto& s = this->_internal_service_name(i);
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      s.data(), static_cast<int>(s.length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.SceneManagerConfig.service_name");
    target = stream->WriteString(1, s, target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.SceneManagerConfig)
  return target;
}

size_t SceneManagerConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.SceneManagerConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated string service_name = 1;
  total_size += 1 *
      ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(service_name_.size());
  for (int i = 0, n = service_name_.size(); i < n; i++) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
      service_name_.Get(i));
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SceneManagerConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SceneManagerConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SceneManagerConfig::GetClassData() const { return &_class_data_; }

void SceneManagerConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SceneManagerConfig *>(to)->MergeFrom(
      static_cast<const SceneManagerConfig &>(from));
}


void SceneManagerConfig::MergeFrom(const SceneManagerConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.SceneManagerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  service_name_.MergeFrom(from.service_name_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SceneManagerConfig::CopyFrom(const SceneManagerConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.SceneManagerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SceneManagerConfig::IsInitialized() const {
  return true;
}

void SceneManagerConfig::InternalSwap(SceneManagerConfig* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  service_name_.InternalSwap(&other->service_name_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SceneManagerConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::SceneManagerConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::SceneManagerConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::SceneManagerConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
