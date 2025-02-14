#ifndef LazyCBS_UTILS_BISECTPERM_HPP_
#define LazyCBS_UTILS_BISECTPERM_HPP_

#include <vector>
#include <map>

namespace LazyCBS {
namespace utils {

/// Generates a Van Der Corput sequence ordering for states to check along an edge.
class BisectPerm
{
public:
  /// Constructor
  BisectPerm();

  /// Destructor
  ~BisectPerm(void);

  /// Returns a map from integer index to fractional position along edge.
  /// For an edge that has n states, we generate a sequence of n fractional
  /// positions along the edge to check for collision, based on Van Der Corput Sequences
  /// We start at 1/2, then 1/4, and 3/4 and so on upto n states.
  /// \param[in] n The number of states along the edge
  const std::vector<std::pair<int,int>>& get(int n);

private:
  std::map<int, const std::vector< std::pair<int,int>>> mCache;
};

} // namespace utils
} // namespace LazyCBS

namespace LazyCBS {
namespace utils {

BisectPerm::BisectPerm()
{
  // Do nothing.
}

BisectPerm::~BisectPerm()
{
  // Do nothing.
}

const std::vector<std::pair<int, int>>& BisectPerm::get(int n)
{
  std::map<int, const std::vector<std::pair<int,int>>>::iterator it;
  it = mCache.find(n);
  if (it != mCache.end())
    return it->second;

  int i;
  int last_true;
  int max_i;
  int max_val;
  std::vector<std::pair<int,int>> perm;
  std::vector<bool> done(n, false);
  std::vector<int> dist(n);

  for (;;)
  {
    last_true = -1;
    for (i=0; i<n; i++)
    {
      if (done[i]) last_true = i;
      dist[i] = (i-last_true);
    }
    last_true = n;
    for (i=n-1; i>=0; i--)
    {
      if (done[i]) last_true = i;
      dist[i] = (last_true-i) < dist[i] ? (last_true-i) : dist[i];
    }
    max_val = 0;
    max_i = 0;
    for (i=0; i<n; i++) if (max_val < dist[i])
    {
      max_val = dist[i];
      max_i = i;
    }
    if (!max_val)
      break;
    perm.push_back(std::make_pair(max_i,max_val));
    done[max_i] = true;
  }

  mCache.insert(std::make_pair(n,perm));
  return mCache[n];
}

} // namespace utils
} // namespace LazyCBS

#endif // LazyCBS_UTILS_BISECTPERM_HPP_
