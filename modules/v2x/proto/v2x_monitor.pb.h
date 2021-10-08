// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/v2x/proto/v2x_monitor.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5fmonitor_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5fmonitor_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fv2x_2fproto_2fv2x_5fmonitor_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fv2x_2fproto_2fv2x_5fmonitor_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fv2x_2fproto_2fv2x_5fmonitor_2eproto;
namespace apollo {
namespace v2x {
class ObuAlarm;
struct ObuAlarmDefaultTypeInternal;
extern ObuAlarmDefaultTypeInternal _ObuAlarm_default_instance_;
}  // namespace v2x
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::v2x::ObuAlarm* Arena::CreateMaybeMessage<::apollo::v2x::ObuAlarm>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace v2x {

enum ErrorCode : int {
  LTEV = 500,
  NET = 501,
  CPU = 502,
  MEM = 503,
  GPS = 504,
  MAP = 510,
  SPAT = 511,
  OBUID = 999
};
bool ErrorCode_IsValid(int value);
constexpr ErrorCode ErrorCode_MIN = LTEV;
constexpr ErrorCode ErrorCode_MAX = OBUID;
constexpr int ErrorCode_ARRAYSIZE = ErrorCode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ErrorCode_descriptor();
template<typename T>
inline const std::string& ErrorCode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, ErrorCode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function ErrorCode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    ErrorCode_descriptor(), enum_t_value);
}
inline bool ErrorCode_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, ErrorCode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<ErrorCode>(
    ErrorCode_descriptor(), name, value);
}
// ===================================================================

class ObuAlarm final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.v2x.ObuAlarm) */ {
 public:
  inline ObuAlarm() : ObuAlarm(nullptr) {}
  ~ObuAlarm() override;
  explicit constexpr ObuAlarm(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ObuAlarm(const ObuAlarm& from);
  ObuAlarm(ObuAlarm&& from) noexcept
    : ObuAlarm() {
    *this = ::std::move(from);
  }

  inline ObuAlarm& operator=(const ObuAlarm& from) {
    CopyFrom(from);
    return *this;
  }
  inline ObuAlarm& operator=(ObuAlarm&& from) noexcept {
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
  static const ObuAlarm& default_instance() {
    return *internal_default_instance();
  }
  static inline const ObuAlarm* internal_default_instance() {
    return reinterpret_cast<const ObuAlarm*>(
               &_ObuAlarm_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ObuAlarm& a, ObuAlarm& b) {
    a.Swap(&b);
  }
  inline void Swap(ObuAlarm* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ObuAlarm* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ObuAlarm* New() const final {
    return new ObuAlarm();
  }

  ObuAlarm* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ObuAlarm>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const ObuAlarm& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const ObuAlarm& from);
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
  void InternalSwap(ObuAlarm* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.v2x.ObuAlarm";
  }
  protected:
  explicit ObuAlarm(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kErrorMsgFieldNumber = 3,
    kModeNumFieldNumber = 1,
    kErrorNumFieldNumber = 2,
  };
  // required string error_msg = 3;
  bool has_error_msg() const;
  private:
  bool _internal_has_error_msg() const;
  public:
  void clear_error_msg();
  const std::string& error_msg() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_error_msg(ArgT0&& arg0, ArgT... args);
  std::string* mutable_error_msg();
  PROTOBUF_MUST_USE_RESULT std::string* release_error_msg();
  void set_allocated_error_msg(std::string* error_msg);
  private:
  const std::string& _internal_error_msg() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_error_msg(const std::string& value);
  std::string* _internal_mutable_error_msg();
  public:

  // required double mode_num = 1;
  bool has_mode_num() const;
  private:
  bool _internal_has_mode_num() const;
  public:
  void clear_mode_num();
  double mode_num() const;
  void set_mode_num(double value);
  private:
  double _internal_mode_num() const;
  void _internal_set_mode_num(double value);
  public:

  // required .apollo.v2x.ErrorCode error_num = 2;
  bool has_error_num() const;
  private:
  bool _internal_has_error_num() const;
  public:
  void clear_error_num();
  ::apollo::v2x::ErrorCode error_num() const;
  void set_error_num(::apollo::v2x::ErrorCode value);
  private:
  ::apollo::v2x::ErrorCode _internal_error_num() const;
  void _internal_set_error_num(::apollo::v2x::ErrorCode value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.v2x.ObuAlarm)
 private:
  class _Internal;

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr error_msg_;
  double mode_num_;
  int error_num_;
  friend struct ::TableStruct_modules_2fv2x_2fproto_2fv2x_5fmonitor_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ObuAlarm

// required double mode_num = 1;
inline bool ObuAlarm::_internal_has_mode_num() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool ObuAlarm::has_mode_num() const {
  return _internal_has_mode_num();
}
inline void ObuAlarm::clear_mode_num() {
  mode_num_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double ObuAlarm::_internal_mode_num() const {
  return mode_num_;
}
inline double ObuAlarm::mode_num() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.ObuAlarm.mode_num)
  return _internal_mode_num();
}
inline void ObuAlarm::_internal_set_mode_num(double value) {
  _has_bits_[0] |= 0x00000002u;
  mode_num_ = value;
}
inline void ObuAlarm::set_mode_num(double value) {
  _internal_set_mode_num(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.ObuAlarm.mode_num)
}

// required .apollo.v2x.ErrorCode error_num = 2;
inline bool ObuAlarm::_internal_has_error_num() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool ObuAlarm::has_error_num() const {
  return _internal_has_error_num();
}
inline void ObuAlarm::clear_error_num() {
  error_num_ = 500;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::apollo::v2x::ErrorCode ObuAlarm::_internal_error_num() const {
  return static_cast< ::apollo::v2x::ErrorCode >(error_num_);
}
inline ::apollo::v2x::ErrorCode ObuAlarm::error_num() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.ObuAlarm.error_num)
  return _internal_error_num();
}
inline void ObuAlarm::_internal_set_error_num(::apollo::v2x::ErrorCode value) {
  assert(::apollo::v2x::ErrorCode_IsValid(value));
  _has_bits_[0] |= 0x00000004u;
  error_num_ = value;
}
inline void ObuAlarm::set_error_num(::apollo::v2x::ErrorCode value) {
  _internal_set_error_num(value);
  // @@protoc_insertion_point(field_set:apollo.v2x.ObuAlarm.error_num)
}

// required string error_msg = 3;
inline bool ObuAlarm::_internal_has_error_msg() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool ObuAlarm::has_error_msg() const {
  return _internal_has_error_msg();
}
inline void ObuAlarm::clear_error_msg() {
  error_msg_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& ObuAlarm::error_msg() const {
  // @@protoc_insertion_point(field_get:apollo.v2x.ObuAlarm.error_msg)
  return _internal_error_msg();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void ObuAlarm::set_error_msg(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000001u;
 error_msg_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:apollo.v2x.ObuAlarm.error_msg)
}
inline std::string* ObuAlarm::mutable_error_msg() {
  std::string* _s = _internal_mutable_error_msg();
  // @@protoc_insertion_point(field_mutable:apollo.v2x.ObuAlarm.error_msg)
  return _s;
}
inline const std::string& ObuAlarm::_internal_error_msg() const {
  return error_msg_.Get();
}
inline void ObuAlarm::_internal_set_error_msg(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  error_msg_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* ObuAlarm::_internal_mutable_error_msg() {
  _has_bits_[0] |= 0x00000001u;
  return error_msg_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* ObuAlarm::release_error_msg() {
  // @@protoc_insertion_point(field_release:apollo.v2x.ObuAlarm.error_msg)
  if (!_internal_has_error_msg()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return error_msg_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void ObuAlarm::set_allocated_error_msg(std::string* error_msg) {
  if (error_msg != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  error_msg_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), error_msg,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:apollo.v2x.ObuAlarm.error_msg)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace v2x
}  // namespace apollo

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::apollo::v2x::ErrorCode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::v2x::ErrorCode>() {
  return ::apollo::v2x::ErrorCode_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fv2x_2fproto_2fv2x_5fmonitor_2eproto
