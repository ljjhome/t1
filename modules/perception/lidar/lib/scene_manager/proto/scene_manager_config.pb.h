// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3018000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3018000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto;
namespace apollo {
namespace perception {
namespace lidar {
class SceneManagerConfig;
struct SceneManagerConfigDefaultTypeInternal;
extern SceneManagerConfigDefaultTypeInternal _SceneManagerConfig_default_instance_;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::perception::lidar::SceneManagerConfig* Arena::CreateMaybeMessage<::apollo::perception::lidar::SceneManagerConfig>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace perception {
namespace lidar {

// ===================================================================

class SceneManagerConfig final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.perception.lidar.SceneManagerConfig) */ {
 public:
  inline SceneManagerConfig() : SceneManagerConfig(nullptr) {}
  ~SceneManagerConfig() override;
  explicit constexpr SceneManagerConfig(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  SceneManagerConfig(const SceneManagerConfig& from);
  SceneManagerConfig(SceneManagerConfig&& from) noexcept
    : SceneManagerConfig() {
    *this = ::std::move(from);
  }

  inline SceneManagerConfig& operator=(const SceneManagerConfig& from) {
    CopyFrom(from);
    return *this;
  }
  inline SceneManagerConfig& operator=(SceneManagerConfig&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const SceneManagerConfig& default_instance() {
    return *internal_default_instance();
  }
  static inline const SceneManagerConfig* internal_default_instance() {
    return reinterpret_cast<const SceneManagerConfig*>(
               &_SceneManagerConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SceneManagerConfig& a, SceneManagerConfig& b) {
    a.Swap(&b);
  }
  inline void Swap(SceneManagerConfig* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(SceneManagerConfig* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SceneManagerConfig* New() const final {
    return new SceneManagerConfig();
  }

  SceneManagerConfig* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SceneManagerConfig>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const SceneManagerConfig& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const SceneManagerConfig& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(SceneManagerConfig* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.perception.lidar.SceneManagerConfig";
  }
  protected:
  explicit SceneManagerConfig(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kServiceNameFieldNumber = 1,
  };
  // repeated string service_name = 1;
  int service_name_size() const;
  private:
  int _internal_service_name_size() const;
  public:
  void clear_service_name();
  const std::string& service_name(int index) const;
  std::string* mutable_service_name(int index);
  void set_service_name(int index, const std::string& value);
  void set_service_name(int index, std::string&& value);
  void set_service_name(int index, const char* value);
  void set_service_name(int index, const char* value, size_t size);
  std::string* add_service_name();
  void add_service_name(const std::string& value);
  void add_service_name(std::string&& value);
  void add_service_name(const char* value);
  void add_service_name(const char* value, size_t size);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>& service_name() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>* mutable_service_name();
  private:
  const std::string& _internal_service_name(int index) const;
  std::string* _internal_add_service_name();
  public:

  // @@protoc_insertion_point(class_scope:apollo.perception.lidar.SceneManagerConfig)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string> service_name_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SceneManagerConfig

// repeated string service_name = 1;
inline int SceneManagerConfig::_internal_service_name_size() const {
  return service_name_.size();
}
inline int SceneManagerConfig::service_name_size() const {
  return _internal_service_name_size();
}
inline void SceneManagerConfig::clear_service_name() {
  service_name_.Clear();
}
inline std::string* SceneManagerConfig::add_service_name() {
  std::string* _s = _internal_add_service_name();
  // @@protoc_insertion_point(field_add_mutable:apollo.perception.lidar.SceneManagerConfig.service_name)
  return _s;
}
inline const std::string& SceneManagerConfig::_internal_service_name(int index) const {
  return service_name_.Get(index);
}
inline const std::string& SceneManagerConfig::service_name(int index) const {
  // @@protoc_insertion_point(field_get:apollo.perception.lidar.SceneManagerConfig.service_name)
  return _internal_service_name(index);
}
inline std::string* SceneManagerConfig::mutable_service_name(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.perception.lidar.SceneManagerConfig.service_name)
  return service_name_.Mutable(index);
}
inline void SceneManagerConfig::set_service_name(int index, const std::string& value) {
  service_name_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline void SceneManagerConfig::set_service_name(int index, std::string&& value) {
  service_name_.Mutable(index)->assign(std::move(value));
  // @@protoc_insertion_point(field_set:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline void SceneManagerConfig::set_service_name(int index, const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  service_name_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline void SceneManagerConfig::set_service_name(int index, const char* value, size_t size) {
  service_name_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline std::string* SceneManagerConfig::_internal_add_service_name() {
  return service_name_.Add();
}
inline void SceneManagerConfig::add_service_name(const std::string& value) {
  service_name_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline void SceneManagerConfig::add_service_name(std::string&& value) {
  service_name_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline void SceneManagerConfig::add_service_name(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  service_name_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline void SceneManagerConfig::add_service_name(const char* value, size_t size) {
  service_name_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:apollo.perception.lidar.SceneManagerConfig.service_name)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>&
SceneManagerConfig::service_name() const {
  // @@protoc_insertion_point(field_list:apollo.perception.lidar.SceneManagerConfig.service_name)
  return service_name_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>*
SceneManagerConfig::mutable_service_name() {
  // @@protoc_insertion_point(field_mutable_list:apollo.perception.lidar.SceneManagerConfig.service_name)
  return &service_name_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fperception_2flidar_2flib_2fscene_5fmanager_2fproto_2fscene_5fmanager_5fconfig_2eproto