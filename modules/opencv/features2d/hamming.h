#pragma once
namespace
{
struct HammingOperator
{
  typedef unsigned char ValueType;

  //! important that this is signed as weird behavior happens
  // in BruteForce if not
  typedef int ResultType;

  unsigned int
  operator()(const unsigned char* a, const unsigned char* b, int size)
  {
#if __GNUC__
    unsigned int result = 0;
    {
      //for portability just use unsigned long -- and use the __builtin_popcountll (see docs for __builtin_popcountll)
      typedef unsigned long long pop_t;
      const size_t modulo = size % sizeof(pop_t);
      const pop_t * a2 = reinterpret_cast<const pop_t*>(a);
      const pop_t * b2 = reinterpret_cast<const pop_t*>(b);

      if (size == sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2));
      if (size == 2 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)));
      if (size == 3 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)))
               + __builtin_popcountll((*(a2 + 2)) ^ (*(b2 + 2)));
      if (size == 4 * sizeof(pop_t))
        return __builtin_popcountll((*a2) ^ (*b2)) + __builtin_popcountll((*(a2 + 1)) ^ (*(b2 + 1)))
               + __builtin_popcountll((*(a2 + 2)) ^ (*(b2 + 2)))
               + __builtin_popcountll((*(a2 + 3)) ^ (*(b2 + 3)));

      const pop_t * a2_end = a2 + (size / sizeof(pop_t));

      for (; a2 != a2_end; ++a2, ++b2)
        result += __builtin_popcountll((*a2) ^ (*b2));

      if (modulo)
      {
        //in the case where size is not divisible by sizeof(size_t)
        //need to mask off the bits at the end
        pop_t a_final = 0, b_final = 0;
        memcpy(&a_final, a2, modulo);
        memcpy(&b_final, b2, modulo);
        result += __builtin_popcountll(a_final ^ b_final);
      }
    }
    return result;
#else
    return cv::HammingLUT()(a,b,size);
#endif
  }
};
}
