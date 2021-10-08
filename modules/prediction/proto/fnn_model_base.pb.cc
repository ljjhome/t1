// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/prediction/proto/fnn_model_base.proto

#include "modules/prediction/proto/fnn_model_base.pb.h"

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
namespace prediction {
constexpr Vector::Vector(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : columns_(){}
struct VectorDefaultTypeInternal {
  constexpr VectorDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~VectorDefaultTypeInternal() {}
  union {
    Vector _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT VectorDefaultTypeInternal _Vector_default_instance_;
constexpr Matrix::Matrix(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : rows_(){}
struct MatrixDefaultTypeInternal {
  constexpr MatrixDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~MatrixDefaultTypeInternal() {}
  union {
    Matrix _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT MatrixDefaultTypeInternal _Matrix_default_instance_;
constexpr Layer::Layer(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : layer_input_weight_(nullptr)
  , layer_bias_(nullptr)
  , layer_input_dim_(0)
  , layer_output_dim_(0)
  , layer_activation_func_(0)
{}
struct LayerDefaultTypeInternal {
  constexpr LayerDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~LayerDefaultTypeInternal() {}
  union {
    Layer _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT LayerDefaultTypeInternal _Layer_default_instance_;
}  // namespace prediction
}  // namespace apollo
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto[3];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Vector, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Vector, columns_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Matrix, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Matrix, rows_),
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Layer, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Layer, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Layer, layer_input_dim_),
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Layer, layer_output_dim_),
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Layer, layer_input_weight_),
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Layer, layer_bias_),
  PROTOBUF_FIELD_OFFSET(::apollo::prediction::Layer, layer_activation_func_),
  2,
  3,
  0,
  1,
  4,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::apollo::prediction::Vector)},
  { 7, -1, -1, sizeof(::apollo::prediction::Matrix)},
  { 14, 25, -1, sizeof(::apollo::prediction::Layer)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::prediction::_Vector_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::prediction::_Matrix_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::prediction::_Layer_default_instance_),
};

const char descriptor_table_protodef_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n-modules/prediction/proto/fnn_model_bas"
  "e.proto\022\021apollo.prediction\"\031\n\006Vector\022\017\n\007"
  "columns\030\001 \003(\001\"1\n\006Matrix\022\'\n\004rows\030\001 \003(\0132\031."
  "apollo.prediction.Vector\"\250\002\n\005Layer\022\027\n\017la"
  "yer_input_dim\030\001 \001(\005\022\030\n\020layer_output_dim\030"
  "\002 \001(\005\0225\n\022layer_input_weight\030\003 \001(\0132\031.apol"
  "lo.prediction.Matrix\022-\n\nlayer_bias\030\004 \001(\013"
  "2\031.apollo.prediction.Vector\022F\n\025layer_act"
  "ivation_func\030\005 \001(\0162\'.apollo.prediction.L"
  "ayer.ActivationFunc\">\n\016ActivationFunc\022\010\n"
  "\004RELU\020\000\022\010\n\004TANH\020\001\022\013\n\007SIGMOID\020\002\022\013\n\007SOFTMA"
  "X\020\003"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto = {
  false, false, 443, descriptor_table_protodef_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto, "modules/prediction/proto/fnn_model_base.proto", 
  &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_once, nullptr, 0, 3,
  schemas, file_default_instances, TableStruct_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto::offsets,
  file_level_metadata_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto, file_level_enum_descriptors_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto, file_level_service_descriptors_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_getter() {
  return &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto(&descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto);
namespace apollo {
namespace prediction {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Layer_ActivationFunc_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto);
  return file_level_enum_descriptors_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto[0];
}
bool Layer_ActivationFunc_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr Layer_ActivationFunc Layer::RELU;
constexpr Layer_ActivationFunc Layer::TANH;
constexpr Layer_ActivationFunc Layer::SIGMOID;
constexpr Layer_ActivationFunc Layer::SOFTMAX;
constexpr Layer_ActivationFunc Layer::ActivationFunc_MIN;
constexpr Layer_ActivationFunc Layer::ActivationFunc_MAX;
constexpr int Layer::ActivationFunc_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

class Vector::_Internal {
 public:
};

Vector::Vector(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  columns_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.prediction.Vector)
}
Vector::Vector(const Vector& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      columns_(from.columns_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.prediction.Vector)
}

void Vector::SharedCtor() {
}

Vector::~Vector() {
  // @@protoc_insertion_point(destructor:apollo.prediction.Vector)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Vector::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Vector::ArenaDtor(void* object) {
  Vector* _this = reinterpret_cast< Vector* >(object);
  (void)_this;
}
void Vector::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Vector::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Vector::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.prediction.Vector)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  columns_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Vector::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated double columns = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          ptr -= 1;
          do {
            ptr += 1;
            _internal_add_columns(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
            ptr += sizeof(double);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<9>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_columns(), ptr, ctx);
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
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Vector::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.prediction.Vector)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated double columns = 1;
  for (int i = 0, n = this->_internal_columns_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_columns(i), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.prediction.Vector)
  return target;
}

size_t Vector::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.prediction.Vector)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double columns = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_columns_size());
    size_t data_size = 8UL * count;
    total_size += 1 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_columns_size());
    total_size += data_size;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Vector::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Vector::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Vector::GetClassData() const { return &_class_data_; }

void Vector::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Vector *>(to)->MergeFrom(
      static_cast<const Vector &>(from));
}


void Vector::MergeFrom(const Vector& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.prediction.Vector)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  columns_.MergeFrom(from.columns_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Vector::CopyFrom(const Vector& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.prediction.Vector)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vector::IsInitialized() const {
  return true;
}

void Vector::InternalSwap(Vector* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  columns_.InternalSwap(&other->columns_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Vector::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_getter, &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_once,
      file_level_metadata_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto[0]);
}

// ===================================================================

class Matrix::_Internal {
 public:
};

Matrix::Matrix(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  rows_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.prediction.Matrix)
}
Matrix::Matrix(const Matrix& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      rows_(from.rows_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.prediction.Matrix)
}

void Matrix::SharedCtor() {
}

Matrix::~Matrix() {
  // @@protoc_insertion_point(destructor:apollo.prediction.Matrix)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Matrix::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Matrix::ArenaDtor(void* object) {
  Matrix* _this = reinterpret_cast< Matrix* >(object);
  (void)_this;
}
void Matrix::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Matrix::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Matrix::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.prediction.Matrix)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  rows_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Matrix::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated .apollo.prediction.Vector rows = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_rows(), ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* Matrix::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.prediction.Matrix)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.prediction.Vector rows = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_rows_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_rows(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.prediction.Matrix)
  return target;
}

size_t Matrix::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.prediction.Matrix)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.prediction.Vector rows = 1;
  total_size += 1UL * this->_internal_rows_size();
  for (const auto& msg : this->rows_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Matrix::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Matrix::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Matrix::GetClassData() const { return &_class_data_; }

void Matrix::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Matrix *>(to)->MergeFrom(
      static_cast<const Matrix &>(from));
}


void Matrix::MergeFrom(const Matrix& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.prediction.Matrix)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  rows_.MergeFrom(from.rows_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Matrix::CopyFrom(const Matrix& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.prediction.Matrix)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Matrix::IsInitialized() const {
  return true;
}

void Matrix::InternalSwap(Matrix* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  rows_.InternalSwap(&other->rows_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Matrix::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_getter, &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_once,
      file_level_metadata_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto[1]);
}

// ===================================================================

class Layer::_Internal {
 public:
  using HasBits = decltype(std::declval<Layer>()._has_bits_);
  static void set_has_layer_input_dim(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_layer_output_dim(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static const ::apollo::prediction::Matrix& layer_input_weight(const Layer* msg);
  static void set_has_layer_input_weight(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::prediction::Vector& layer_bias(const Layer* msg);
  static void set_has_layer_bias(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_layer_activation_func(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

const ::apollo::prediction::Matrix&
Layer::_Internal::layer_input_weight(const Layer* msg) {
  return *msg->layer_input_weight_;
}
const ::apollo::prediction::Vector&
Layer::_Internal::layer_bias(const Layer* msg) {
  return *msg->layer_bias_;
}
Layer::Layer(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:apollo.prediction.Layer)
}
Layer::Layer(const Layer& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_layer_input_weight()) {
    layer_input_weight_ = new ::apollo::prediction::Matrix(*from.layer_input_weight_);
  } else {
    layer_input_weight_ = nullptr;
  }
  if (from._internal_has_layer_bias()) {
    layer_bias_ = new ::apollo::prediction::Vector(*from.layer_bias_);
  } else {
    layer_bias_ = nullptr;
  }
  ::memcpy(&layer_input_dim_, &from.layer_input_dim_,
    static_cast<size_t>(reinterpret_cast<char*>(&layer_activation_func_) -
    reinterpret_cast<char*>(&layer_input_dim_)) + sizeof(layer_activation_func_));
  // @@protoc_insertion_point(copy_constructor:apollo.prediction.Layer)
}

void Layer::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&layer_input_weight_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&layer_activation_func_) -
    reinterpret_cast<char*>(&layer_input_weight_)) + sizeof(layer_activation_func_));
}

Layer::~Layer() {
  // @@protoc_insertion_point(destructor:apollo.prediction.Layer)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Layer::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete layer_input_weight_;
  if (this != internal_default_instance()) delete layer_bias_;
}

void Layer::ArenaDtor(void* object) {
  Layer* _this = reinterpret_cast< Layer* >(object);
  (void)_this;
}
void Layer::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Layer::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Layer::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.prediction.Layer)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(layer_input_weight_ != nullptr);
      layer_input_weight_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(layer_bias_ != nullptr);
      layer_bias_->Clear();
    }
  }
  if (cached_has_bits & 0x0000001cu) {
    ::memset(&layer_input_dim_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&layer_activation_func_) -
        reinterpret_cast<char*>(&layer_input_dim_)) + sizeof(layer_activation_func_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Layer::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional int32 layer_input_dim = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_layer_input_dim(&has_bits);
          layer_input_dim_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional int32 layer_output_dim = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_layer_output_dim(&has_bits);
          layer_output_dim_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.prediction.Matrix layer_input_weight = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_layer_input_weight(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.prediction.Vector layer_bias = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_layer_bias(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .apollo.prediction.Layer.ActivationFunc layer_activation_func = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::prediction::Layer_ActivationFunc_IsValid(val))) {
            _internal_set_layer_activation_func(static_cast<::apollo::prediction::Layer_ActivationFunc>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(5, val, mutable_unknown_fields());
          }
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

::PROTOBUF_NAMESPACE_ID::uint8* Layer::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.prediction.Layer)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 layer_input_dim = 1;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(1, this->_internal_layer_input_dim(), target);
  }

  // optional int32 layer_output_dim = 2;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_layer_output_dim(), target);
  }

  // optional .apollo.prediction.Matrix layer_input_weight = 3;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::layer_input_weight(this), target, stream);
  }

  // optional .apollo.prediction.Vector layer_bias = 4;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        4, _Internal::layer_bias(this), target, stream);
  }

  // optional .apollo.prediction.Layer.ActivationFunc layer_activation_func = 5;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      5, this->_internal_layer_activation_func(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.prediction.Layer)
  return target;
}

size_t Layer::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.prediction.Layer)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional .apollo.prediction.Matrix layer_input_weight = 3;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *layer_input_weight_);
    }

    // optional .apollo.prediction.Vector layer_bias = 4;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *layer_bias_);
    }

    // optional int32 layer_input_dim = 1;
    if (cached_has_bits & 0x00000004u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_layer_input_dim());
    }

    // optional int32 layer_output_dim = 2;
    if (cached_has_bits & 0x00000008u) {
      total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_layer_output_dim());
    }

    // optional .apollo.prediction.Layer.ActivationFunc layer_activation_func = 5;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_layer_activation_func());
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Layer::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Layer::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Layer::GetClassData() const { return &_class_data_; }

void Layer::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Layer *>(to)->MergeFrom(
      static_cast<const Layer &>(from));
}


void Layer::MergeFrom(const Layer& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.prediction.Layer)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_layer_input_weight()->::apollo::prediction::Matrix::MergeFrom(from._internal_layer_input_weight());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_layer_bias()->::apollo::prediction::Vector::MergeFrom(from._internal_layer_bias());
    }
    if (cached_has_bits & 0x00000004u) {
      layer_input_dim_ = from.layer_input_dim_;
    }
    if (cached_has_bits & 0x00000008u) {
      layer_output_dim_ = from.layer_output_dim_;
    }
    if (cached_has_bits & 0x00000010u) {
      layer_activation_func_ = from.layer_activation_func_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Layer::CopyFrom(const Layer& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.prediction.Layer)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Layer::IsInitialized() const {
  return true;
}

void Layer::InternalSwap(Layer* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Layer, layer_activation_func_)
      + sizeof(Layer::layer_activation_func_)
      - PROTOBUF_FIELD_OFFSET(Layer, layer_input_weight_)>(
          reinterpret_cast<char*>(&layer_input_weight_),
          reinterpret_cast<char*>(&other->layer_input_weight_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Layer::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_getter, &descriptor_table_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto_once,
      file_level_metadata_modules_2fprediction_2fproto_2ffnn_5fmodel_5fbase_2eproto[2]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace prediction
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::prediction::Vector* Arena::CreateMaybeMessage< ::apollo::prediction::Vector >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::prediction::Vector >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::prediction::Matrix* Arena::CreateMaybeMessage< ::apollo::prediction::Matrix >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::prediction::Matrix >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::prediction::Layer* Arena::CreateMaybeMessage< ::apollo::prediction::Layer >(Arena* arena) {
  return Arena::CreateMessageInternal< ::apollo::prediction::Layer >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
