// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cyber/proto/proto_desc.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fproto_5fdesc_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fproto_5fdesc_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_cyber_2fproto_2fproto_5fdesc_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_cyber_2fproto_2fproto_5fdesc_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cyber_2fproto_2fproto_5fdesc_2eproto;
namespace apollo {
namespace cyber {
namespace proto {
class ProtoDesc;
struct ProtoDescDefaultTypeInternal;
extern ProtoDescDefaultTypeInternal _ProtoDesc_default_instance_;
}  // namespace proto
}  // namespace cyber
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::cyber::proto::ProtoDesc* Arena::CreateMaybeMessage<::apollo::cyber::proto::ProtoDesc>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace cyber {
namespace proto {

// ===================================================================

class ProtoDesc final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.cyber.proto.ProtoDesc) */ {
 public:
  inline ProtoDesc() : ProtoDesc(nullptr) {}
  ~ProtoDesc() override;
  explicit constexpr ProtoDesc(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ProtoDesc(const ProtoDesc& from);
  ProtoDesc(ProtoDesc&& from) noexcept
    : ProtoDesc() {
    *this = ::std::move(from);
  }

  inline ProtoDesc& operator=(const ProtoDesc& from) {
    CopyFrom(from);
    return *this;
  }
  inline ProtoDesc& operator=(ProtoDesc&& from) noexcept {
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
  static const ProtoDesc& default_instance() {
    return *internal_default_instance();
  }
  static inline const ProtoDesc* internal_default_instance() {
    return reinterpret_cast<const ProtoDesc*>(
               &_ProtoDesc_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ProtoDesc& a, ProtoDesc& b) {
    a.Swap(&b);
  }
  inline void Swap(ProtoDesc* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ProtoDesc* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ProtoDesc* New() const final {
    return new ProtoDesc();
  }

  ProtoDesc* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ProtoDesc>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const ProtoDesc& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const ProtoDesc& from);
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
  void InternalSwap(ProtoDesc* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.cyber.proto.ProtoDesc";
  }
  protected:
  explicit ProtoDesc(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kDependenciesFieldNumber = 2,
    kDescFieldNumber = 1,
  };
  // repeated .apollo.cyber.proto.ProtoDesc dependencies = 2;
  int dependencies_size() const;
  private:
  int _internal_dependencies_size() const;
  public:
  void clear_dependencies();
  ::apollo::cyber::proto::ProtoDesc* mutable_dependencies(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::cyber::proto::ProtoDesc >*
      mutable_dependencies();
  private:
  const ::apollo::cyber::proto::ProtoDesc& _internal_dependencies(int index) const;
  ::apollo::cyber::proto::ProtoDesc* _internal_add_dependencies();
  public:
  const ::apollo::cyber::proto::ProtoDesc& dependencies(int index) const;
  ::apollo::cyber::proto::ProtoDesc* add_dependencies();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::cyber::proto::ProtoDesc >&
      dependencies() const;

  // optional bytes desc = 1;
  bool has_desc() const;
  private:
  bool _internal_has_desc() const;
  public:
  void clear_desc();
  const std::string& desc() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_desc(ArgT0&& arg0, ArgT... args);
  std::string* mutable_desc();
  PROTOBUF_MUST_USE_RESULT std::string* release_desc();
  void set_allocated_desc(std::string* desc);
  private:
  const std::string& _internal_desc() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_desc(const std::string& value);
  std::string* _internal_mutable_desc();
  public:

  // @@protoc_insertion_point(class_scope:apollo.cyber.proto.ProtoDesc)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::cyber::proto::ProtoDesc > dependencies_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr desc_;
  friend struct ::TableStruct_cyber_2fproto_2fproto_5fdesc_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ProtoDesc

// optional bytes desc = 1;
inline bool ProtoDesc::_internal_has_desc() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool ProtoDesc::has_desc() const {
  return _internal_has_desc();
}
inline void ProtoDesc::clear_desc() {
  desc_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& ProtoDesc::desc() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.ProtoDesc.desc)
  return _internal_desc();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void ProtoDesc::set_desc(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000001u;
 desc_.SetBytes(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.ProtoDesc.desc)
}
inline std::string* ProtoDesc::mutable_desc() {
  std::string* _s = _internal_mutable_desc();
  // @@protoc_insertion_point(field_mutable:apollo.cyber.proto.ProtoDesc.desc)
  return _s;
}
inline const std::string& ProtoDesc::_internal_desc() const {
  return desc_.Get();
}
inline void ProtoDesc::_internal_set_desc(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  desc_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* ProtoDesc::_internal_mutable_desc() {
  _has_bits_[0] |= 0x00000001u;
  return desc_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* ProtoDesc::release_desc() {
  // @@protoc_insertion_point(field_release:apollo.cyber.proto.ProtoDesc.desc)
  if (!_internal_has_desc()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return desc_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void ProtoDesc::set_allocated_desc(std::string* desc) {
  if (desc != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  desc_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), desc,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:apollo.cyber.proto.ProtoDesc.desc)
}

// repeated .apollo.cyber.proto.ProtoDesc dependencies = 2;
inline int ProtoDesc::_internal_dependencies_size() const {
  return dependencies_.size();
}
inline int ProtoDesc::dependencies_size() const {
  return _internal_dependencies_size();
}
inline void ProtoDesc::clear_dependencies() {
  dependencies_.Clear();
}
inline ::apollo::cyber::proto::ProtoDesc* ProtoDesc::mutable_dependencies(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.cyber.proto.ProtoDesc.dependencies)
  return dependencies_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::cyber::proto::ProtoDesc >*
ProtoDesc::mutable_dependencies() {
  // @@protoc_insertion_point(field_mutable_list:apollo.cyber.proto.ProtoDesc.dependencies)
  return &dependencies_;
}
inline const ::apollo::cyber::proto::ProtoDesc& ProtoDesc::_internal_dependencies(int index) const {
  return dependencies_.Get(index);
}
inline const ::apollo::cyber::proto::ProtoDesc& ProtoDesc::dependencies(int index) const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.ProtoDesc.dependencies)
  return _internal_dependencies(index);
}
inline ::apollo::cyber::proto::ProtoDesc* ProtoDesc::_internal_add_dependencies() {
  return dependencies_.Add();
}
inline ::apollo::cyber::proto::ProtoDesc* ProtoDesc::add_dependencies() {
  ::apollo::cyber::proto::ProtoDesc* _add = _internal_add_dependencies();
  // @@protoc_insertion_point(field_add:apollo.cyber.proto.ProtoDesc.dependencies)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::cyber::proto::ProtoDesc >&
ProtoDesc::dependencies() const {
  // @@protoc_insertion_point(field_list:apollo.cyber.proto.ProtoDesc.dependencies)
  return dependencies_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace cyber
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fproto_5fdesc_2eproto
