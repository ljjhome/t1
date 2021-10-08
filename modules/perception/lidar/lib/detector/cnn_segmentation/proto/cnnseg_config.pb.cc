// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/detector/cnn_segmentation/proto/cnnseg_config.proto

#include "modules/perception/lidar/lib/detector/cnn_segmentation/proto/cnnseg_config.pb.h"

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
constexpr CNNSegConfig::CNNSegConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : param_file_(nullptr)
  , proto_file_(nullptr)
  , weight_file_(nullptr)
  , engine_file_(nullptr){}
struct CNNSegConfigDefaultTypeInternal {
  constexpr CNNSegConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~CNNSegConfigDefaultTypeInternal() {}
  union {
    CNNSegConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT CNNSegConfigDefaultTypeInternal _CNNSegConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CNNSegConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CNNSegConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CNNSegConfig, param_file_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CNNSegConfig, proto_file_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CNNSegConfig, weight_file_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CNNSegConfig, engine_file_),
  0,
  1,
  2,
  3,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, -1, sizeof(::apollo::perception::lidar::CNNSegConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_CNNSegConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\nPmodules/perception/lidar/lib/detector/"
  "cnn_segmentation/proto/cnnseg_config.pro"
  "to\022\027apollo.perception.lidar\"\361\001\n\014CNNSegCo"
  "nfig\0223\n\nparam_file\030\001 \001(\t:\037./data/models/"
  "cnnseg/param.conf\0228\n\nproto_file\030\002 \001(\t:$."
  "/data/models/cnnseg/deploy.prototxt\022;\n\013w"
  "eight_file\030\003 \001(\t:&./data/models/cnnseg/d"
  "eploy.caffemodel\0225\n\013engine_file\030\004 \001(\t: ."
  "/data/models/cnnseg/engine.conf"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto = {
  false, false, 351, descriptor_table_protodef_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto, "modules/perception/lidar/lib/detector/cnn_segmentation/proto/cnnseg_config.proto", 
  &descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto(&descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class CNNSegConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<CNNSegConfig>()._has_bits_);
  static void set_has_param_file(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_proto_file(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_weight_file(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_engine_file(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

const ::PROTOBUF_NAMESPACE_ID::internal::LazyString CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_{{{"./data/models/cnnseg/param.conf", 31}}, {nullptr}};
const ::PROTOBUF_NAMESPACE_ID::internal::LazyString CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_{{{"./data/models/cnnseg/deploy.prototxt", 36}}, {nullptr}};
const ::PROTOBUF_NAMESPACE_ID::internal::LazyString CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_{{{"./data/models/cnnseg/deploy.caffemodel", 38}}, {nullptr}};
const ::PROTOBUF_NAMESPACE_ID::internal::LazyString CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_{{{"./data/models/cnnseg/engine.conf", 32}}, {nullptr}};
CNNSegConfig::CNNSegConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.CNNSegConfig)
}
CNNSegConfig::CNNSegConfig(const CNNSegConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  param_file_.UnsafeSetDefault(nullptr);
  if (from._internal_has_param_file()) {
    param_file_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_param_file(), 
      GetArenaForAllocation());
  }
  proto_file_.UnsafeSetDefault(nullptr);
  if (from._internal_has_proto_file()) {
    proto_file_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_proto_file(), 
      GetArenaForAllocation());
  }
  weight_file_.UnsafeSetDefault(nullptr);
  if (from._internal_has_weight_file()) {
    weight_file_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_weight_file(), 
      GetArenaForAllocation());
  }
  engine_file_.UnsafeSetDefault(nullptr);
  if (from._internal_has_engine_file()) {
    engine_file_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::NonEmptyDefault{}, from._internal_engine_file(), 
      GetArenaForAllocation());
  }
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.CNNSegConfig)
}

void CNNSegConfig::SharedCtor() {
param_file_.UnsafeSetDefault(nullptr);
proto_file_.UnsafeSetDefault(nullptr);
weight_file_.UnsafeSetDefault(nullptr);
engine_file_.UnsafeSetDefault(nullptr);
}

CNNSegConfig::~CNNSegConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.CNNSegConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void CNNSegConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  param_file_.DestroyNoArena(nullptr);
  proto_file_.DestroyNoArena(nullptr);
  weight_file_.DestroyNoArena(nullptr);
  engine_file_.DestroyNoArena(nullptr);
}

void CNNSegConfig::ArenaDtor(void* object) {
  CNNSegConfig* _this = reinterpret_cast< CNNSegConfig* >(object);
  (void)_this;
}
void CNNSegConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void CNNSegConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void CNNSegConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.CNNSegConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      param_file_.ClearToDefault(::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_param_file_, GetArenaForAllocation());
       }
    if (cached_has_bits & 0x00000002u) {
      proto_file_.ClearToDefault(::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_proto_file_, GetArenaForAllocation());
       }
    if (cached_has_bits & 0x00000004u) {
      weight_file_.ClearToDefault(::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_weight_file_, GetArenaForAllocation());
       }
    if (cached_has_bits & 0x00000008u) {
      engine_file_.ClearToDefault(::apollo::perception::lidar::CNNSegConfig::_i_give_permission_to_break_this_code_default_engine_file_, GetArenaForAllocation());
       }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CNNSegConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional string param_file = 1 [default = "./data/models/cnnseg/param.conf"];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_param_file();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.CNNSegConfig.param_file");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string proto_file = 2 [default = "./data/models/cnnseg/deploy.prototxt"];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_proto_file();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.CNNSegConfig.proto_file");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string weight_file = 3 [default = "./data/models/cnnseg/deploy.caffemodel"];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          auto str = _internal_mutable_weight_file();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.CNNSegConfig.weight_file");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string engine_file = 4 [default = "./data/models/cnnseg/engine.conf"];
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          auto str = _internal_mutable_engine_file();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.CNNSegConfig.engine_file");
          #endif  // !NDEBUG
          CHK_(ptr);
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
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* CNNSegConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.CNNSegConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string param_file = 1 [default = "./data/models/cnnseg/param.conf"];
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_param_file().data(), static_cast<int>(this->_internal_param_file().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.CNNSegConfig.param_file");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_param_file(), target);
  }

  // optional string proto_file = 2 [default = "./data/models/cnnseg/deploy.prototxt"];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_proto_file().data(), static_cast<int>(this->_internal_proto_file().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.CNNSegConfig.proto_file");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_proto_file(), target);
  }

  // optional string weight_file = 3 [default = "./data/models/cnnseg/deploy.caffemodel"];
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_weight_file().data(), static_cast<int>(this->_internal_weight_file().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.CNNSegConfig.weight_file");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_weight_file(), target);
  }

  // optional string engine_file = 4 [default = "./data/models/cnnseg/engine.conf"];
  if (cached_has_bits & 0x00000008u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_engine_file().data(), static_cast<int>(this->_internal_engine_file().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.CNNSegConfig.engine_file");
    target = stream->WriteStringMaybeAliased(
        4, this->_internal_engine_file(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.CNNSegConfig)
  return target;
}

size_t CNNSegConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.CNNSegConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional string param_file = 1 [default = "./data/models/cnnseg/param.conf"];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_param_file());
    }

    // optional string proto_file = 2 [default = "./data/models/cnnseg/deploy.prototxt"];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_proto_file());
    }

    // optional string weight_file = 3 [default = "./data/models/cnnseg/deploy.caffemodel"];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_weight_file());
    }

    // optional string engine_file = 4 [default = "./data/models/cnnseg/engine.conf"];
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_engine_file());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData CNNSegConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    CNNSegConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*CNNSegConfig::GetClassData() const { return &_class_data_; }

void CNNSegConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<CNNSegConfig *>(to)->MergeFrom(
      static_cast<const CNNSegConfig &>(from));
}


void CNNSegConfig::MergeFrom(const CNNSegConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.CNNSegConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_param_file(from._internal_param_file());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_set_proto_file(from._internal_proto_file());
    }
    if (cached_has_bits & 0x00000004u) {
      _internal_set_weight_file(from._internal_weight_file());
    }
    if (cached_has_bits & 0x00000008u) {
      _internal_set_engine_file(from._internal_engine_file());
    }
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void CNNSegConfig::CopyFrom(const CNNSegConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.CNNSegConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CNNSegConfig::IsInitialized() const {
  return true;
}

void CNNSegConfig::InternalSwap(CNNSegConfig* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &param_file_, lhs_arena,
      &other->param_file_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &proto_file_, lhs_arena,
      &other->proto_file_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &weight_file_, lhs_arena,
      &other->weight_file_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      nullptr,
      &engine_file_, lhs_arena,
      &other->engine_file_, rhs_arena
  );
}

::PROTOBUF_NAMESPACE_ID::Metadata CNNSegConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2flidar_2flib_2fdetector_2fcnn_5fsegmentation_2fproto_2fcnnseg_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::CNNSegConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::CNNSegConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::CNNSegConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
