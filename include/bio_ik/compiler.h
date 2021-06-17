#pragma once

#include <cstddef>

#ifdef __GNUC__
# define BIO_IK_FORCE_INLINE __attribute__((always_inline))
#elif defined(_MSC_VER)
# define BIO_IK_FORCE_INLINE  __forceinline
#else
# define BIO_IK_FORCE_INLINE
#endif

namespace bio_ik
{

template<size_t>
struct SignedForSize
{};

template<>
struct SignedForSize<2>
{
    using type = int16_t;
};

template<>
struct SignedForSize<4>
{
    using type = int32_t;
};

template<>
struct SignedForSize<8>
{
    using type = int64_t;
};

#ifndef __linux__
using ssize_t = typename SignedForSize<sizeof(size_t)>::type;
#endif



#ifdef _WIN32
BIO_IK_FORCE_INLINE int check_align(size_t align)
{
    for (size_t i = sizeof(void *); i != 0; i *= 2)
    if (align == i)
        return 0;
    return EINVAL;
}

BIO_IK_FORCE_INLINE int posix_memalign(void **ptr, size_t align, size_t size)
{
    if (check_align(align))
        return EINVAL;

    int saved_errno = errno;
    void *p = _aligned_malloc(size, align);
    if (p == NULL)
    {
        errno = saved_errno;
        return ENOMEM;
    }

    *ptr = p;
    return 0;
}
#endif

}

#ifdef __GNUC__
# define BIO_IK_LIKELY(x) (__builtin__expect((x), 1))
# define BIO_IK_UNLIKELY(x) (__builtin__expect((x), 0))
#else
# define BIO_IK_LIKELY(x) (x)
# define BIO_IK_UNLIKELY(x) (x)
#endif 

#ifdef __GNUC__
# define BIO_IK_RESTRICT __restrict__
#elif defined(_MSC_VER)
# define BIO_IK_RESTRICT __restrict
#else
# define BIO_IK_RESTRICT
#endif

#ifdef _WIN32
# ifdef BIO_IK_BUILDING
#  define BIO_IK_EXTERN __declspec(dllexport)
# else // BIO_IK_BUILDING
#  define BIO_IK_EXTERN __declspec(dllimport)
# endif // BIO_IK_BUILDING
#else // _WIN32
# define BIO_IK_EXTERN
#endif // _WIN32
