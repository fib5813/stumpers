  int maxArea(vector<int>& height) {
        int start = 0;
        int end = height.size()-1;
        int out = 0;
        while(start != end){
            int area = (end-start)*(min(height.at(start), height.at(end)));
            out = max(area, out);
            if(height.at(start) < height.at(end)) start++;
            else end--;
        }
     return out;
    }
        
    // int maxArea(vector<int>& height) {
    //     int out = 0;
    //     int start = 0;
    //     while(start < height.size()-1){
    //         int end = start + 1;
    //         while(end < height.size()){
    //             int new_out = (end-start)*min(height.at(start), height.at(end));
    //             if( new_out > out){
    //                 out = new_out;
    //             }
    //             end++;
    //         }
    //         start++;
    //     }
    //     return out;
    // }