// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/proto/ccrf_type_fusion_config.proto

#include "modules/perception/proto/ccrf_type_fusion_config.pb.h"

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
constexpr CcrfTypeFusionConfig::CcrfTypeFusionConfig(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : classifiers_property_file_path_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , transition_property_file_path_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , transition_matrix_alpha_(1.8f){}
struct CcrfTypeFusionConfigDefaultTypeInternal {
  constexpr CcrfTypeFusionConfigDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~CcrfTypeFusionConfigDefaultTypeInternal() {}
  union {
    CcrfTypeFusionConfig _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT CcrfTypeFusionConfigDefaultTypeInternal _CcrfTypeFusionConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CcrfTypeFusionConfig, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CcrfTypeFusionConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CcrfTypeFusionConfig, classifiers_property_file_path_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CcrfTypeFusionConfig, transition_property_file_path_),
  PROTOBUF_FIELD_OFFSET(::apollo::perception::lidar::CcrfTypeFusionConfig, transition_matrix_alpha_),
  0,
  1,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, -1, sizeof(::apollo::perception::lidar::CcrfTypeFusionConfig)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::perception::lidar::_CcrfTypeFusionConfig_default_instance_),
};

const char descriptor_table_protodef_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n6modules/perception/proto/ccrf_type_fus"
  "ion_config.proto\022\027apollo.perception.lida"
  "r\"\217\001\n\024CcrfTypeFusionConfig\022(\n\036classifier"
  "s_property_file_path\030\001 \001(\t:\000\022\'\n\035transiti"
  "on_property_file_path\030\002 \001(\t:\000\022$\n\027transit"
  "ion_matrix_alpha\030\003 \001(\002:\0031.8"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto = {
  false, false, 227, descriptor_table_protodef_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto, "modules/perception/proto/ccrf_type_fusion_config.proto", 
  &descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto_once, nullptr, 0, 1,
  schemas, file_default_instances, TableStruct_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto::offsets,
  file_level_metadata_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto, file_level_enum_descriptors_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto, file_level_service_descriptors_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto_getter() {
  return &descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto(&descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto);
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class CcrfTypeFusionConfig::_Internal {
 public:
  using HasBits = decltype(std::declval<CcrfTypeFusionConfig>()._has_bits_);
  static void set_has_classifiers_property_file_path(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_transition_property_file_path(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_transition_matrix_alpha(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

CcrfTypeFusionConfig::CcrfTypeFusionConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.perception.lidar.CcrfTypeFusionConfig)
}
CcrfTypeFusionConfig::CcrfTypeFusionConfig(const CcrfTypeFusionConfig& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  classifiers_property_file_path_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_classifiers_property_file_path()) {
    classifiers_property_file_path_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_classifiers_property_file_path(), 
      GetArenaForAllocation());
  }
  transition_property_file_path_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_transition_property_file_path()) {
    transition_property_file_path_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_transition_property_file_path(), 
      GetArenaForAllocation());
  }
  transition_matrix_alpha_ = from.transition_matrix_alpha_;
  // @@protoc_insertion_point(copy_constructor:apollo.perception.lidar.CcrfTypeFusionConfig)
}

void CcrfTypeFusionConfig::SharedCtor() {
classifiers_property_file_path_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
transition_property_file_path_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
transition_matrix_alpha_ = 1.8f;
}

CcrfTypeFusionConfig::~CcrfTypeFusionConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.lidar.CcrfTypeFusionConfig)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void CcrfTypeFusionConfig::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  classifiers_property_file_path_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  transition_property_file_path_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void CcrfTypeFusionConfig::ArenaDtor(void* object) {
  CcrfTypeFusionConfig* _this = reinterpret_cast< CcrfTypeFusionConfig* >(object);
  (void)_this;
}
void CcrfTypeFusionConfig::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void CcrfTypeFusionConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void CcrfTypeFusionConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.lidar.CcrfTypeFusionConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      classifiers_property_file_path_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000002u) {
      transition_property_file_path_.ClearNonDefaultToEmpty();
    }
    transition_matrix_alpha_ = 1.8f;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CcrfTypeFusionConfig::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional string classifiers_property_file_path = 1 [default = ""];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_classifiers_property_file_path();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string transition_property_file_path = 2 [default = ""];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_transition_property_file_path();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional float transition_matrix_alpha = 3 [default = 1.8];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 29)) {
          _Internal::set_has_transition_matrix_alpha(&has_bits);
          transition_matrix_alpha_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
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

::PROTOBUF_NAMESPACE_ID::uint8* CcrfTypeFusionConfig::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.lidar.CcrfTypeFusionConfig)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string classifiers_property_file_path = 1 [default = ""];
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_classifiers_property_file_path().data(), static_cast<int>(this->_internal_classifiers_property_file_path().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.CcrfTypeFusionConfig.classifiers_property_file_path");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_classifiers_property_file_path(), target);
  }

  // optional string transition_property_file_path = 2 [default = ""];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_transition_property_file_path().data(), static_cast<int>(this->_internal_transition_property_file_path().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.perception.lidar.CcrfTypeFusionConfig.transition_property_file_path");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_transition_property_file_path(), target);
  }

  // optional float transition_matrix_alpha = 3 [default = 1.8];
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_transition_matrix_alpha(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.lidar.CcrfTypeFusionConfig)
  return target;
}

size_t CcrfTypeFusionConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.lidar.CcrfTypeFusionConfig)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional string classifiers_property_file_path = 1 [default = ""];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_classifiers_property_file_path());
    }

    // optional string transition_property_file_path = 2 [default = ""];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_transition_property_file_path());
    }

    // optional float transition_matrix_alpha = 3 [default = 1.8];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 4;
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData CcrfTypeFusionConfig::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    CcrfTypeFusionConfig::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*CcrfTypeFusionConfig::GetClassData() const { return &_class_data_; }

void CcrfTypeFusionConfig::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<CcrfTypeFusionConfig *>(to)->MergeFrom(
      static_cast<const CcrfTypeFusionConfig &>(from));
}


void CcrfTypeFusionConfig::MergeFrom(const CcrfTypeFusionConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.lidar.CcrfTypeFusionConfig)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_classifiers_property_file_path(from._internal_classifiers_property_file_path());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_set_transition_property_file_path(from._internal_transition_property_file_path());
    }
    if (cached_has_bits & 0x00000004u) {
      transition_matrix_alpha_ = from.transition_matrix_alpha_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void CcrfTypeFusionConfig::CopyFrom(const CcrfTypeFusionConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.lidar.CcrfTypeFusionConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CcrfTypeFusionConfig::IsInitialized() const {
  return true;
}

void CcrfTypeFusionConfig::InternalSwap(CcrfTypeFusionConfig* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &classifiers_property_file_path_, lhs_arena,
      &other->classifiers_property_file_path_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &transition_property_file_path_, lhs_arena,
      &other->transition_property_file_path_, rhs_arena
  );
  swap(transition_matrix_alpha_, other->transition_matrix_alpha_);
}

::PROTOBUF_NAMESPACE_ID::Metadata CcrfTypeFusionConfig::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto_getter, &descriptor_table_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto_once,
      file_level_metadata_modules_2fperception_2fproto_2fccrf_5ftype_5ffusion_5fconfig_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::perception::lidar::CcrfTypeFusionConfig* Arena::CreateMaybeMessage< ::apollo::perception::lidar::CcrfTypeFusionConfig >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::perception::lidar::CcrfTypeFusionConfig >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
