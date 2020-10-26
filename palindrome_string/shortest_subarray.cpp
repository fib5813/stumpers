// working:
// int shortestSubarray(vector<int>& a, int K) {
//         int left = 0, right = 0, sum = a.at(left);
//         int min_array = std::numeric_limits<int>::max();
        
//         for (int i = 0; i < a.size(); i++){
//             sum = a.at(i);
//             int j = i;
//             while(sum < K){
//                 if(j+1 < a.size()){
//                     sum = sum+a.at(j+1);
//                     j++;
                    
//                 }
//                 else break;
//             }
//             if(sum >= K) min_array = min(min_array, j-i+1);
//         }
//         if(min_array == std::numeric_limits<int>::max()) return -1;
//         else return min_array;
        
//     }