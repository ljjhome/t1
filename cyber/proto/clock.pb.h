// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cyber/proto/clock.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fclock_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fclock_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_cyber_2fproto_2fclock_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_cyber_2fproto_2fclock_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cyber_2fproto_2fclock_2eproto;
namespace apollo {
namespace cyber {
namespace proto {
class Clock;
struct ClockDefaultTypeInternal;
extern ClockDefaultTypeInternal _Clock_default_instance_;
}  // namespace proto
}  // namespace cyber
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::cyber::proto::Clock* Arena::CreateMaybeMessage<::apollo::cyber::proto::Clock>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace cyber {
namespace proto {

// ===================================================================

class Clock final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.cyber.proto.Clock) */ {
 public:
  inline Clock() : Clock(nullptr) {}
  ~Clock() override;
  explicit constexpr Clock(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Clock(const Clock& from);
  Clock(Clock&& from) noexcept
    : Clock() {
    *this = ::std::move(from);
  }

  inline Clock& operator=(const Clock& from) {
    CopyFrom(from);
    return *this;
  }
  inline Clock& operator=(Clock&& from) noexcept {
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
  static const Clock& default_instance() {
    return *internal_default_instance();
  }
  static inline const Clock* internal_default_instance() {
    return reinterpret_cast<const Clock*>(
               &_Clock_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Clock& a, Clock& b) {
    a.Swap(&b);
  }
  inline void Swap(Clock* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Clock* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Clock* New() const final {
    return new Clock();
  }

  Clock* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Clock>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Clock& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Clock& from);
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
  void InternalSwap(Clock* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.cyber.proto.Clock";
  }
  protected:
  explicit Clock(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kClockFieldNumber = 1,
  };
  // required uint64 clock = 1;
  bool has_clock() const;
  private:
  bool _internal_has_clock() const;
  public:
  void clear_clock();
  ::PROTOBUF_NAMESPACE_ID::uint64 clock() const;
  void set_clock(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_clock() const;
  void _internal_set_clock(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.cyber.proto.Clock)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::uint64 clock_;
  friend struct ::TableStruct_cyber_2fproto_2fclock_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Clock

// required uint64 clock = 1;
inline bool Clock::_internal_has_clock() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Clock::has_clock() const {
  return _internal_has_clock();
}
inline void Clock::clear_clock() {
  clock_ = uint64_t{0u};
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Clock::_internal_clock() const {
  return clock_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Clock::clock() const {
  // @@protoc_insertion_point(field_get:apollo.cyber.proto.Clock.clock)
  return _internal_clock();
}
inline void Clock::_internal_set_clock(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000001u;
  clock_ = value;
}
inline void Clock::set_clock(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_clock(value);
  // @@protoc_insertion_point(field_set:apollo.cyber.proto.Clock.clock)
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
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_cyber_2fproto_2fclock_2eproto
