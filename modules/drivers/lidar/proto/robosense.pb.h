// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/drivers/lidar/proto/robosense.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto

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
#include "modules/common/proto/header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto;
namespace apollo {
namespace drivers {
namespace robosense {
class RobosenseScan;
struct RobosenseScanDefaultTypeInternal;
extern RobosenseScanDefaultTypeInternal _RobosenseScan_default_instance_;
class RobosenseScanPacket;
struct RobosenseScanPacketDefaultTypeInternal;
extern RobosenseScanPacketDefaultTypeInternal _RobosenseScanPacket_default_instance_;
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::drivers::robosense::RobosenseScan* Arena::CreateMaybeMessage<::apollo::drivers::robosense::RobosenseScan>(Arena*);
template<> ::apollo::drivers::robosense::RobosenseScanPacket* Arena::CreateMaybeMessage<::apollo::drivers::robosense::RobosenseScanPacket>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace drivers {
namespace robosense {

// ===================================================================

class RobosenseScanPacket final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.robosense.RobosenseScanPacket) */ {
 public:
  inline RobosenseScanPacket() : RobosenseScanPacket(nullptr) {}
  ~RobosenseScanPacket() override;
  explicit constexpr RobosenseScanPacket(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  RobosenseScanPacket(const RobosenseScanPacket& from);
  RobosenseScanPacket(RobosenseScanPacket&& from) noexcept
    : RobosenseScanPacket() {
    *this = ::std::move(from);
  }

  inline RobosenseScanPacket& operator=(const RobosenseScanPacket& from) {
    CopyFrom(from);
    return *this;
  }
  inline RobosenseScanPacket& operator=(RobosenseScanPacket&& from) noexcept {
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
  static const RobosenseScanPacket& default_instance() {
    return *internal_default_instance();
  }
  static inline const RobosenseScanPacket* internal_default_instance() {
    return reinterpret_cast<const RobosenseScanPacket*>(
               &_RobosenseScanPacket_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RobosenseScanPacket& a, RobosenseScanPacket& b) {
    a.Swap(&b);
  }
  inline void Swap(RobosenseScanPacket* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(RobosenseScanPacket* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RobosenseScanPacket* New() const final {
    return new RobosenseScanPacket();
  }

  RobosenseScanPacket* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RobosenseScanPacket>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const RobosenseScanPacket& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const RobosenseScanPacket& from);
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
  void InternalSwap(RobosenseScanPacket* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.drivers.robosense.RobosenseScanPacket";
  }
  protected:
  explicit RobosenseScanPacket(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kDataFieldNumber = 2,
    kStampFieldNumber = 1,
  };
  // optional bytes data = 2;
  bool has_data() const;
  private:
  bool _internal_has_data() const;
  public:
  void clear_data();
  const std::string& data() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_data(ArgT0&& arg0, ArgT... args);
  std::string* mutable_data();
  PROTOBUF_MUST_USE_RESULT std::string* release_data();
  void set_allocated_data(std::string* data);
  private:
  const std::string& _internal_data() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_data(const std::string& value);
  std::string* _internal_mutable_data();
  public:

  // optional uint64 stamp = 1;
  bool has_stamp() const;
  private:
  bool _internal_has_stamp() const;
  public:
  void clear_stamp();
  ::PROTOBUF_NAMESPACE_ID::uint64 stamp() const;
  void set_stamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_stamp() const;
  void _internal_set_stamp(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.drivers.robosense.RobosenseScanPacket)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr data_;
  ::PROTOBUF_NAMESPACE_ID::uint64 stamp_;
  friend struct ::TableStruct_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto;
};
// -------------------------------------------------------------------

class RobosenseScan final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.drivers.robosense.RobosenseScan) */ {
 public:
  inline RobosenseScan() : RobosenseScan(nullptr) {}
  ~RobosenseScan() override;
  explicit constexpr RobosenseScan(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  RobosenseScan(const RobosenseScan& from);
  RobosenseScan(RobosenseScan&& from) noexcept
    : RobosenseScan() {
    *this = ::std::move(from);
  }

  inline RobosenseScan& operator=(const RobosenseScan& from) {
    CopyFrom(from);
    return *this;
  }
  inline RobosenseScan& operator=(RobosenseScan&& from) noexcept {
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
  static const RobosenseScan& default_instance() {
    return *internal_default_instance();
  }
  static inline const RobosenseScan* internal_default_instance() {
    return reinterpret_cast<const RobosenseScan*>(
               &_RobosenseScan_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(RobosenseScan& a, RobosenseScan& b) {
    a.Swap(&b);
  }
  inline void Swap(RobosenseScan* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(RobosenseScan* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RobosenseScan* New() const final {
    return new RobosenseScan();
  }

  RobosenseScan* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RobosenseScan>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const RobosenseScan& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const RobosenseScan& from);
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
  void InternalSwap(RobosenseScan* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.drivers.robosense.RobosenseScan";
  }
  protected:
  explicit RobosenseScan(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kFiringPktsFieldNumber = 3,
    kModelFieldNumber = 2,
    kHeaderFieldNumber = 1,
    kBasetimeFieldNumber = 4,
  };
  // repeated .apollo.drivers.robosense.RobosenseScanPacket firing_pkts = 3;
  int firing_pkts_size() const;
  private:
  int _internal_firing_pkts_size() const;
  public:
  void clear_firing_pkts();
  ::apollo::drivers::robosense::RobosenseScanPacket* mutable_firing_pkts(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::drivers::robosense::RobosenseScanPacket >*
      mutable_firing_pkts();
  private:
  const ::apollo::drivers::robosense::RobosenseScanPacket& _internal_firing_pkts(int index) const;
  ::apollo::drivers::robosense::RobosenseScanPacket* _internal_add_firing_pkts();
  public:
  const ::apollo::drivers::robosense::RobosenseScanPacket& firing_pkts(int index) const;
  ::apollo::drivers::robosense::RobosenseScanPacket* add_firing_pkts();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::drivers::robosense::RobosenseScanPacket >&
      firing_pkts() const;

  // optional string model = 2;
  bool has_model() const;
  private:
  bool _internal_has_model() const;
  public:
  void clear_model();
  const std::string& model() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_model(ArgT0&& arg0, ArgT... args);
  std::string* mutable_model();
  PROTOBUF_MUST_USE_RESULT std::string* release_model();
  void set_allocated_model(std::string* model);
  private:
  const std::string& _internal_model() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_model(const std::string& value);
  std::string* _internal_mutable_model();
  public:

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

  // optional uint64 basetime = 4 [default = 0];
  bool has_basetime() const;
  private:
  bool _internal_has_basetime() const;
  public:
  void clear_basetime();
  ::PROTOBUF_NAMESPACE_ID::uint64 basetime() const;
  void set_basetime(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_basetime() const;
  void _internal_set_basetime(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // @@protoc_insertion_point(class_scope:apollo.drivers.robosense.RobosenseScan)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::drivers::robosense::RobosenseScanPacket > firing_pkts_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr model_;
  ::apollo::common::Header* header_;
  ::PROTOBUF_NAMESPACE_ID::uint64 basetime_;
  friend struct ::TableStruct_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RobosenseScanPacket

// optional uint64 stamp = 1;
inline bool RobosenseScanPacket::_internal_has_stamp() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool RobosenseScanPacket::has_stamp() const {
  return _internal_has_stamp();
}
inline void RobosenseScanPacket::clear_stamp() {
  stamp_ = uint64_t{0u};
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 RobosenseScanPacket::_internal_stamp() const {
  return stamp_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 RobosenseScanPacket::stamp() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.robosense.RobosenseScanPacket.stamp)
  return _internal_stamp();
}
inline void RobosenseScanPacket::_internal_set_stamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000002u;
  stamp_ = value;
}
inline void RobosenseScanPacket::set_stamp(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_stamp(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.robosense.RobosenseScanPacket.stamp)
}

// optional bytes data = 2;
inline bool RobosenseScanPacket::_internal_has_data() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool RobosenseScanPacket::has_data() const {
  return _internal_has_data();
}
inline void RobosenseScanPacket::clear_data() {
  data_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& RobosenseScanPacket::data() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.robosense.RobosenseScanPacket.data)
  return _internal_data();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void RobosenseScanPacket::set_data(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000001u;
 data_.SetBytes(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:apollo.drivers.robosense.RobosenseScanPacket.data)
}
inline std::string* RobosenseScanPacket::mutable_data() {
  std::string* _s = _internal_mutable_data();
  // @@protoc_insertion_point(field_mutable:apollo.drivers.robosense.RobosenseScanPacket.data)
  return _s;
}
inline const std::string& RobosenseScanPacket::_internal_data() const {
  return data_.Get();
}
inline void RobosenseScanPacket::_internal_set_data(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  data_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* RobosenseScanPacket::_internal_mutable_data() {
  _has_bits_[0] |= 0x00000001u;
  return data_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* RobosenseScanPacket::release_data() {
  // @@protoc_insertion_point(field_release:apollo.drivers.robosense.RobosenseScanPacket.data)
  if (!_internal_has_data()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return data_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void RobosenseScanPacket::set_allocated_data(std::string* data) {
  if (data != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  data_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), data,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.robosense.RobosenseScanPacket.data)
}

// -------------------------------------------------------------------

// RobosenseScan

// optional .apollo.common.Header header = 1;
inline bool RobosenseScan::_internal_has_header() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || header_ != nullptr);
  return value;
}
inline bool RobosenseScan::has_header() const {
  return _internal_has_header();
}
inline const ::apollo::common::Header& RobosenseScan::_internal_header() const {
  const ::apollo::common::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::apollo::common::Header&>(
      ::apollo::common::_Header_default_instance_);
}
inline const ::apollo::common::Header& RobosenseScan::header() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.robosense.RobosenseScan.header)
  return _internal_header();
}
inline void RobosenseScan::unsafe_arena_set_allocated_header(
    ::apollo::common::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:apollo.drivers.robosense.RobosenseScan.header)
}
inline ::apollo::common::Header* RobosenseScan::release_header() {
  _has_bits_[0] &= ~0x00000002u;
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
inline ::apollo::common::Header* RobosenseScan::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:apollo.drivers.robosense.RobosenseScan.header)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::apollo::common::Header* RobosenseScan::_internal_mutable_header() {
  _has_bits_[0] |= 0x00000002u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::apollo::common::Header* RobosenseScan::mutable_header() {
  ::apollo::common::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:apollo.drivers.robosense.RobosenseScan.header)
  return _msg;
}
inline void RobosenseScan::set_allocated_header(::apollo::common::Header* header) {
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
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.robosense.RobosenseScan.header)
}

// optional string model = 2;
inline bool RobosenseScan::_internal_has_model() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool RobosenseScan::has_model() const {
  return _internal_has_model();
}
inline void RobosenseScan::clear_model() {
  model_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& RobosenseScan::model() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.robosense.RobosenseScan.model)
  return _internal_model();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void RobosenseScan::set_model(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000001u;
 model_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:apollo.drivers.robosense.RobosenseScan.model)
}
inline std::string* RobosenseScan::mutable_model() {
  std::string* _s = _internal_mutable_model();
  // @@protoc_insertion_point(field_mutable:apollo.drivers.robosense.RobosenseScan.model)
  return _s;
}
inline const std::string& RobosenseScan::_internal_model() const {
  return model_.Get();
}
inline void RobosenseScan::_internal_set_model(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  model_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* RobosenseScan::_internal_mutable_model() {
  _has_bits_[0] |= 0x00000001u;
  return model_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* RobosenseScan::release_model() {
  // @@protoc_insertion_point(field_release:apollo.drivers.robosense.RobosenseScan.model)
  if (!_internal_has_model()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return model_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void RobosenseScan::set_allocated_model(std::string* model) {
  if (model != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  model_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), model,
      GetArenaForAllocation());
  // @@protoc_insertion_point(field_set_allocated:apollo.drivers.robosense.RobosenseScan.model)
}

// repeated .apollo.drivers.robosense.RobosenseScanPacket firing_pkts = 3;
inline int RobosenseScan::_internal_firing_pkts_size() const {
  return firing_pkts_.size();
}
inline int RobosenseScan::firing_pkts_size() const {
  return _internal_firing_pkts_size();
}
inline void RobosenseScan::clear_firing_pkts() {
  firing_pkts_.Clear();
}
inline ::apollo::drivers::robosense::RobosenseScanPacket* RobosenseScan::mutable_firing_pkts(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.drivers.robosense.RobosenseScan.firing_pkts)
  return firing_pkts_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::drivers::robosense::RobosenseScanPacket >*
RobosenseScan::mutable_firing_pkts() {
  // @@protoc_insertion_point(field_mutable_list:apollo.drivers.robosense.RobosenseScan.firing_pkts)
  return &firing_pkts_;
}
inline const ::apollo::drivers::robosense::RobosenseScanPacket& RobosenseScan::_internal_firing_pkts(int index) const {
  return firing_pkts_.Get(index);
}
inline const ::apollo::drivers::robosense::RobosenseScanPacket& RobosenseScan::firing_pkts(int index) const {
  // @@protoc_insertion_point(field_get:apollo.drivers.robosense.RobosenseScan.firing_pkts)
  return _internal_firing_pkts(index);
}
inline ::apollo::drivers::robosense::RobosenseScanPacket* RobosenseScan::_internal_add_firing_pkts() {
  return firing_pkts_.Add();
}
inline ::apollo::drivers::robosense::RobosenseScanPacket* RobosenseScan::add_firing_pkts() {
  ::apollo::drivers::robosense::RobosenseScanPacket* _add = _internal_add_firing_pkts();
  // @@protoc_insertion_point(field_add:apollo.drivers.robosense.RobosenseScan.firing_pkts)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::drivers::robosense::RobosenseScanPacket >&
RobosenseScan::firing_pkts() const {
  // @@protoc_insertion_point(field_list:apollo.drivers.robosense.RobosenseScan.firing_pkts)
  return firing_pkts_;
}

// optional uint64 basetime = 4 [default = 0];
inline bool RobosenseScan::_internal_has_basetime() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool RobosenseScan::has_basetime() const {
  return _internal_has_basetime();
}
inline void RobosenseScan::clear_basetime() {
  basetime_ = uint64_t{0u};
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 RobosenseScan::_internal_basetime() const {
  return basetime_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 RobosenseScan::basetime() const {
  // @@protoc_insertion_point(field_get:apollo.drivers.robosense.RobosenseScan.basetime)
  return _internal_basetime();
}
inline void RobosenseScan::_internal_set_basetime(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000004u;
  basetime_ = value;
}
inline void RobosenseScan::set_basetime(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_basetime(value);
  // @@protoc_insertion_point(field_set:apollo.drivers.robosense.RobosenseScan.basetime)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fdrivers_2flidar_2fproto_2frobosense_2eproto