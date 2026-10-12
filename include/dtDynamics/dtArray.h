/*!
 \file      dtArray.h
 \brief     Lightweight package-owned dynamic array container.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTARRAY_H_
#define ALGORITHM_DTDYNAMICS_DTARRAY_H_

#include <cstddef>
#include <initializer_list>
#include <new>
#include <type_traits>
#include <utility>

namespace dt::Dynamics
{

namespace detail
{

/*! \brief Convert an integral array size to std::size_t, clamping signed non-positive values to zero. */
template <typename Size>
std::size_t ToArraySize(const Size value)
{
    if constexpr (std::is_signed<Size>::value)
    {
        return value > 0 ? static_cast<std::size_t>(value) : static_cast<std::size_t>(0);
    }
    else
    {
        return static_cast<std::size_t>(value);
    }
}

} // namespace detail

/*!
 \brief Package-owned dynamic array for topology, names, and per-body/per-dof structured storage.
 \details Dense numerical vector and matrix storage stays in dtMath. dtArray only covers
          variable-count package data where dtMath does not provide the needed container.
          The API intentionally mirrors the subset of std::vector used by the public
          dtDynamics model/data structures while keeping the public dependency boundary small.

          dtArray owns a contiguous T[] allocation and value-initializes newly reserved
          storage through new T[n]. It is not a realtime container: reserve/growth, clear,
          resize beyond capacity, copy assignment, and initializer-list assignment may
          allocate and move/copy elements. Element access is unchecked; callers must pass
          indices in the half-open range [0, size()).
*/
template <typename T>
class dtArray
{
public:
    /*! \brief Construct an empty array with no allocation. */
    dtArray() = default;

    /*! \brief Construct by copying values from an initializer list. */
    dtArray(std::initializer_list<T> values)
    {
        try
        {
            reserve(values.size());
            for (const T &value : values)
            {
                push_back(value);
            }
        }
        catch (...)
        {
            clear();
            throw;
        }
    }

    /*! \brief Copy-construct by appending each element from other. */
    dtArray(const dtArray &other)
    {
        try
        {
            reserve(other.m_size);
            for (std::size_t i = 0; i < other.m_size; ++i)
            {
                push_back(other.m_data[i]);
            }
        }
        catch (...)
        {
            clear();
            throw;
        }
    }

    /*! \brief Move-construct, leaving other empty. */
    dtArray(dtArray &&other) noexcept
        : m_data(other.m_data), m_size(other.m_size), m_capacity(other.m_capacity)
    {
        other.m_data = nullptr;
        other.m_size = 0;
        other.m_capacity = 0;
    }

    /*! \brief Destroy owned storage. */
    ~dtArray()
    {
        delete[] m_data;
    }

    /*! \brief Copy-assign using copy-and-swap semantics. */
    dtArray &operator=(const dtArray &other)
    {
        if (this == &other)
        {
            return *this;
        }

        dtArray copy(other);
        swap(copy);
        return *this;
    }

    /*! \brief Move-assign, releasing current storage and leaving other empty. */
    dtArray &operator=(dtArray &&other) noexcept
    {
        if (this == &other)
        {
            return *this;
        }

        delete[] m_data;
        m_data = other.m_data;
        m_size = other.m_size;
        m_capacity = other.m_capacity;
        other.m_data = nullptr;
        other.m_size = 0;
        other.m_capacity = 0;
        return *this;
    }

    /*! \brief Replace contents by copying values from an initializer list. */
    dtArray &operator=(std::initializer_list<T> values)
    {
        clear();
        reserve(values.size());
        for (const T &value : values)
        {
            push_back(value);
        }
        return *this;
    }

    /*! \brief Release all storage and reset size/capacity to zero. */
    void clear()
    {
        delete[] m_data;
        m_data = nullptr;
        m_size = 0;
        m_capacity = 0;
    }

    /*! \brief Change the logical size, growing capacity when needed.
        \details New elements, when capacity grows, come from value-initialized T[] storage.
                 Shrinking does not destroy individual elements until storage is released.
        \note Regrowing within existing capacity re-exposes prior values rather than
              value-initializing those elements. */
    void resize(const std::size_t size)
    {
        if (size > m_capacity)
        {
            reserve(size);
        }
        m_size = size;
    }

    /*! \brief Integral-size overload for resize; signed non-positive sizes become zero. */
    template <typename Size,
              typename = std::enable_if_t<std::is_integral<Size>::value &&
                                          !std::is_same<Size, bool>::value>>
    void resize(const Size size)
    {
        resize(detail::ToArraySize(size));
    }

    /*! \brief Resize to size and assign value to every logical element. */
    void assign(const std::size_t size, const T &value)
    {
        resize(size);
        for (std::size_t i = 0; i < m_size; ++i)
        {
            m_data[i] = value;
        }
    }

    /*! \brief Integral-size overload for assign; signed non-positive sizes become zero. */
    template <typename Size,
              typename = std::enable_if_t<std::is_integral<Size>::value &&
                                          !std::is_same<Size, bool>::value>>
    void assign(const Size size, const T &value)
    {
        assign(detail::ToArraySize(size), value);
    }

    /*! \brief Append a by-value capture before growth can invalidate aliased storage.
        \details Retains the public by-value signature, then hands the captured value to a
                 private mutable-reference implementation. A noexcept move constructor
                 reconstructs the unused allocated slot; otherwise copy assignment or noexcept
                 move assignment provides the destination transfer. */
    void push_back(T value)
    {
        AppendCaptured(value);
    }

    /*! \brief Return true when size() is zero. */
    bool empty() const { return m_size == 0; }
    /*! \brief Return the number of logical elements. */
    std::size_t size() const { return m_size; }

    /*! \brief Return a mutable pointer to contiguous storage, or nullptr for an empty unallocated array. */
    T *data() { return m_data; }
    /*! \brief Return an immutable pointer to contiguous storage, or nullptr for an empty unallocated array. */
    const T *data() const { return m_data; }

    /*! \brief Return element index without bounds checking.
        \pre index must be less than size(). */
    T &operator[](const std::size_t index) { return m_data[index]; }
    /*! \brief Return element index without bounds checking.
        \pre index must be less than size(). */
    const T &operator[](const std::size_t index) const { return m_data[index]; }

    /*! \brief Return a mutable iterator/pointer to the first element. */
    T *begin() { return m_data; }
    /*! \brief Return an immutable iterator/pointer to the first element. */
    const T *begin() const { return m_data; }
    /*! \brief Return a mutable one-past-end iterator/pointer, or nullptr when unallocated. */
    T *end() { return m_data == nullptr ? nullptr : m_data + m_size; }
    /*! \brief Return an immutable one-past-end iterator/pointer, or nullptr when unallocated. */
    const T *end() const { return m_data == nullptr ? nullptr : m_data + m_size; }

private:
    /*! \brief Assign a captured value while preserving mutable-copy overload selection. */
    static void CopyIntoSlot(T &destination, T &source)
    {
        destination = source;
    }

    /*! \brief Complete append after push_back has captured any potentially aliased input. */
    void AppendCaptured(T &value)
    {
        if (m_size == m_capacity)
        {
            reserve(m_capacity == 0 ? 1 : 2 * m_capacity);
        }
        if constexpr (std::is_nothrow_move_constructible<T>::value)
        {
            m_data[m_size].~T();
            ::new (static_cast<void *>(m_data + m_size)) T(std::move(value));
        }
        else if constexpr (std::is_copy_assignable<T>::value)
        {
            CopyIntoSlot(m_data[m_size], value);
        }
        else
        {
            static_assert(std::is_nothrow_move_assignable<T>::value,
                          "dtArray append requires a noexcept move constructor, copy assignment, "
                          "or noexcept move assignment");
            m_data[m_size] = std::move(value);
        }
        ++m_size;
    }

    /*! \brief Ensure capacity is at least capacity, preserving existing logical elements.
        \details Elements must be copy-assignable or noexcept move-assignable so failed growth
                 never leaves a move-only source partially transferred. */
    void reserve(const std::size_t capacity)
    {
        if (capacity <= m_capacity)
        {
            return;
        }

        T *const newData = new T[capacity];
        try
        {
            for (std::size_t i = 0; i < m_size; ++i)
            {
                if constexpr (std::is_copy_assignable<T>::value)
                {
                    newData[i] = m_data[i];
                }
                else
                {
                    static_assert(std::is_nothrow_move_assignable<T>::value,
                                  "dtArray growth requires copy-assignable or noexcept move-assignable elements");
                    newData[i] = std::move(m_data[i]);
                }
            }
        }
        catch (...)
        {
            delete[] newData;
            throw;
        }
        delete[] m_data;
        m_data = newData;
        m_capacity = capacity;
    }

    /*! \brief Swap storage metadata with other without allocating. */
    void swap(dtArray &other) noexcept
    {
        T *const data = m_data;
        const std::size_t size = m_size;
        const std::size_t capacity = m_capacity;
        m_data = other.m_data;
        m_size = other.m_size;
        m_capacity = other.m_capacity;
        other.m_data = data;
        other.m_size = size;
        other.m_capacity = capacity;
    }

    T *m_data = nullptr;
    std::size_t m_size = 0;
    std::size_t m_capacity = 0;
};

/*! \brief Backward-compatible alias for package dynamic arrays. */
template <typename T>
using dtDynArray = dtArray<T>;

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTARRAY_H_
