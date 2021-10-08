// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/audio/proto/audio.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2faudio_2fproto_2faudio_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2faudio_2fproto_2faudio_2eproto

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
#include "modules/audio/proto/audio_common.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/proto/header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2faudio_2fproto_2faudio_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2faudio_2fproto_2faudio_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2faudio_2fproto_2faudio_2eproto;
namespace apollo {
namespace audio {
class AudioDetection;
struct AudioDetectionDefaultTypeInternal;
extern AudioDetectionDefaultTypeInternal _AudioDetection_default_instance_;
}  // namespace audio
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::audio::AudioDetection* Arena::CreateMaybeMessage<::apollo::audio::AudioDetection>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace audio {

// ===================================================================

class AudioDetection final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.audio.AudioDetection) */ {
 public:
  inline AudioDetection() : AudioDetection(nullptr) {}
  ~AudioDetection() override;
  explicit constexpr AudioDetection(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  AudioDetection(const AudioDetection& from);
  AudioDetection(AudioDetection&& from) noexcept
    : AudioDetection() {
    *this = ::std::move(from);
  }

  inline AudioDetection& operator=(const AudioDetection& from) {
    CopyFrom(from);
    return *this;
  }
  inline AudioDetection& operator=(AudioDetection&& from) noexcept {
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
  static const AudioDetection& default_instance() {
    return *internal_default_instance();
  }
  static inline const AudioDetection* internal_default_instance() {
    return reinterpret_cast<const AudioDetection*>(
               &_AudioDetection_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(AudioDetection& a, AudioDetection& b) {
    a.Swap(&b);
  }
  inline void Swap(AudioDetection* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(AudioDetection* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline AudioDetection* New() const final {
    return new AudioDetection();
  }

  AudioDetection* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<AudioDetection>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const AudioDetection& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const AudioDetection& from);
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
  void InternalSwap(AudioDetection* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.audio.AudioDetection";
  }
  protected:
  explicit AudioDetection(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kHeaderFieldNumber = 1,
    kPositionFieldNumber = 4,
    kIsSirenFieldNumber = 2,
    kMovingResultFieldNumber = 3,
    kSourceDegreeFieldNumber = 5,
  };
  // optional .apollo.common.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::apollo::common::Header& header() const;
  PROTOBUF_MUST_USE_RESULT ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);
  private:
  const ::apollo::common::Header& _internal_header() const;
  ::apollo::common::Header* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::apollo::common::Header* header);
  ::apollo::common::Header* unsafe_arena_release_header();

  // optional .apollo.common.Point3D position = 4;
  bool has_position() const;
  private:
  bool _internal_has_position() const;
  public:
  void clear_position();
  const ::apollo::common::Point3D& position() const;
  PROTOBUF_MUST_USE_RESULT ::apollo::common::Point3D* release_position();
  ::apollo::common::Point3D* mutable_position();
  void set_allocated_position(::apollo::common::Point3D* position);
  private:
  const ::apollo::common::Point3D& _internal_position() const;
  ::apollo::common::Point3D* _internal_mutable_position();
  public:
  void unsafe_arena_set_allocated_position(
      ::apollo::common::Point3D* position);
  ::apollo::common::Point3D* unsafe_arena_release_position();

  // optional bool is_siren = 2;
  bool has_is_siren() const;
  private:
  bool _internal_has_is_siren() const;
  public:
  void clear_is_siren();
  bool is_siren() const;
  void set_is_siren(bool value);
  private:
  bool _internal_is_siren() const;
  void _internal_set_is_siren(bool value);
  public:

  // optional .apollo.audio.MovingResult moving_result = 3 [default = UNKNOWN];
  bool has_moving_result() const;
  private:
  bool _internal_has_moving_result() const;
  public:
  void clear_moving_result();
  ::apollo::audio::MovingResult moving_result() const;
  void set_moving_result(::apollo::audio::MovingResult value);
  private:
  ::apollo::audio::MovingResult _internal_moving_result() const;
  void _internal_set_moving_result(::apollo::audio::MovingResult value);
  public:

  // optional double source_degree = 5;
  bool has_source_degree() const;
  private:
  bool _internal_has_source_degree() const;
  public:
  void clear_source_degree();
  double source_degree() const;
  void set_source_degree(double value);
  private:
  double _internal_source_degree() const;
  void _internal_set_source_degree(double value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.audio.AudioDetection)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::apollo::common::Header* header_;
  ::apollo::common::Point3D* position_;
  bool is_siren_;
  int moving_result_;
  double source_degree_;
  friend struct ::TableStruct_modules_2faudio_2fproto_2faudio_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// AudioDetection

// optional .apollo.common.Header header = 1;
inline bool AudioDetection::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool AudioDetection::has_header() const {
  return _internal_has_header();
}
inline const ::apollo::common::Header& AudioDetection::_internal_header() const {
  const ::apollo::common::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::common::Header&>(
      ::apollo::common::_Header_default_instance_);
}
inline const ::apollo::common::Header& AudioDetection::header() const {
  // @@protoc_insertion_point(field_get:apollo.audio.AudioDetection.header)
  return _internal_header();
}
inline void AudioDetection::unsafe_arena_set_allocated_header(
    ::apollo::common::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.audio.AudioDetection.header)
}
inline ::apollo::common::Header* AudioDetection::release_header() {
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::apollo::common::Header* AudioDetection::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:apollo.audio.AudioDetection.header)
  _has_bits_[0] &= ~0x00000001u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::common::Header* AudioDetection::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000001u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::apollo::common::Header* AudioDetection::mutable_header() {
  ::apollo::common::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:apollo.audio.AudioDetection.header)
  return _msg;
}
inline void AudioDetection::set_allocated_header(::apollo::common::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header));
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.audio.AudioDetection.header)
}

// optional bool is_siren = 2;
inline bool AudioDetection::_internal_has_is_siren() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool AudioDetection::has_is_siren() const {
  return _internal_has_is_siren();
}
inline void AudioDetection::clear_is_siren() {
  is_siren_ = false;
  _has_bits_[0] &= ~0x00000004u;
}
inline bool AudioDetection::_internal_is_siren() const {
  return is_siren_;
}
inline bool AudioDetection::is_siren() const {
  // @@protoc_insertion_point(field_get:apollo.audio.AudioDetection.is_siren)
  return _internal_is_siren();
}
inline void AudioDetection::_internal_set_is_siren(bool value) {
  _has_bits_[0] |= 0x00000004u;
  is_siren_ = value;
}
inline void AudioDetection::set_is_siren(bool value) {
  _internal_set_is_siren(value);
  // @@protoc_insertion_point(field_set:apollo.audio.AudioDetection.is_siren)
}

// optional .apollo.audio.MovingResult moving_result = 3 [default = UNKNOWN];
inline bool AudioDetection::_internal_has_moving_result() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool AudioDetection::has_moving_result() const {
  return _internal_has_moving_result();
}
inline void AudioDetection::clear_moving_result() {
  moving_result_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::apollo::audio::MovingResult AudioDetection::_internal_moving_result() const {
  return static_cast< ::apollo::audio::MovingResult >(moving_result_);
}
inline ::apollo::audio::MovingResult AudioDetection::moving_result() const {
  // @@protoc_insertion_point(field_get:apollo.audio.AudioDetection.moving_result)
  return _internal_moving_result();
}
inline void AudioDetection::_internal_set_moving_result(::apollo::audio::MovingResult value) {
  assert(::apollo::audio::MovingResult_IsValid(value));
  _has_bits_[0] |= 0x00000008u;
  moving_result_ = value;
}
inline void AudioDetection::set_moving_result(::apollo::audio::MovingResult value) {
  _internal_set_moving_result(value);
  // @@protoc_insertion_point(field_set:apollo.audio.AudioDetection.moving_result)
}

// optional .apollo.common.Point3D position = 4;
inline bool AudioDetection::_internal_has_position() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || position_ != nullptr);
  return value;
}
inline bool AudioDetection::has_position() const {
  return _internal_has_position();
}
inline const ::apollo::common::Point3D& AudioDetection::_internal_position() const {
  const ::apollo::common::Point3D* p = position_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::common::Point3D&>(
      ::apollo::common::_Point3D_default_instance_);
}
inline const ::apollo::common::Point3D& AudioDetection::position() const {
  // @@protoc_insertion_point(field_get:apollo.audio.AudioDetection.position)
  return _internal_position();
}
inline void AudioDetection::unsafe_arena_set_allocated_position(
    ::apollo::common::Point3D* position) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(position_);
  }
  position_ = position;
  if (position) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.audio.AudioDetection.position)
}
inline ::apollo::common::Point3D* AudioDetection::release_position() {
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Point3D* temp = position_;
  position_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::apollo::common::Point3D* AudioDetection::unsafe_arena_release_position() {
  // @@protoc_insertion_point(field_release:apollo.audio.AudioDetection.position)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Point3D* temp = position_;
  position_ = nullptr;
  return temp;
}
inline ::apollo::common::Point3D* AudioDetection::_internal_mutable_position() {
  _has_bits_[0] |= 0x00000002u;
  if (position_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Point3D>(GetArenaForAllocation());
    position_ = p;
  }
  return position_;
}
inline ::apollo::common::Point3D* AudioDetection::mutable_position() {
  ::apollo::common::Point3D* _msg = _internal_mutable_position();
  // @@protoc_insertion_point(field_mutable:apollo.audio.AudioDetection.position)
  return _msg;
}
inline void AudioDetection::set_allocated_position(::apollo::common::Point3D* position) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(position_);
  }
  if (position) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(position));
    if (message_arena != submessage_arena) {
      position = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, position, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  position_ = position;
  // @@protoc_insertion_point(field_set_allocated:apollo.audio.AudioDetection.position)
}

// optional double source_degree = 5;
inline bool AudioDetection::_internal_has_source_degree() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool AudioDetection::has_source_degree() const {
  return _internal_has_source_degree();
}
inline void AudioDetection::clear_source_degree() {
  source_degree_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline double AudioDetection::_internal_source_degree() const {
  return source_degree_;
}
inline double AudioDetection::source_degree() const {
  // @@protoc_insertion_point(field_get:apollo.audio.AudioDetection.source_degree)
  return _internal_source_degree();
}
inline void AudioDetection::_internal_set_source_degree(double value) {
  _has_bits_[0] |= 0x00000010u;
  source_degree_ = value;
}
inline void AudioDetection::set_source_degree(double value) {
  _internal_set_source_degree(value);
  // @@protoc_insertion_point(field_set:apollo.audio.AudioDetection.source_degree)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace audio
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2faudio_2fproto_2faudio_2eproto
