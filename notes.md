# C++ 
1. std::next(a.begin(), 36); std::prev(a.end());
2. smart pointers @see https://stackoverflow.com/a/106614/14007680, https://docs.microsoft.com/en-us/cpp/cpp/smart-pointers-modern-cpp?view=vs-2019
3. singleton design pattern @see https://stackoverflow.com/a/1008289/14007680, https://refactoringguru.cn/design-patterns/singleton/cpp/example, https://gist.github.com/pazdera/1098119
4. Is default constructor necessary? @see https://www.interviewsansar.com/is-it-necessary-to-write-empty-constructor-in-cplusplus/
5. vector::at
6. multi-threading: 
   1. atomic, memory load order, @see https://zhuanlan.zhihu.com/p/107092432
   2. 

# SLAM 
1. ratio of baseline and depth shall not be too small to get a stable reconstruction. 
2. If the camera matrices are known, the fundamental / essential matrix can be computed directly from the epipolar constraint. 
3. 