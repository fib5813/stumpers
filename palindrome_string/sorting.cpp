std::vector<int> test_vector(int n, int min = -10, int max = 10){
    std::vector<int> out{};
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> distr(min, max);
    for(int i = 0; i < n; ++i){
        out.push_back(distr(generator));
    }
    return out;
}

void print(std::vector<int>& in){
    for (auto& i : in){
        std::cout << i << " ";
    }
    std::cout << std::endl;
}

//swap function
void swap(std::vector<int> & in, int one, int two){
    int temp = in.at(one);
    in.at(one) = in.at(two);
    in.at(two) = temp;
}


// sort in place
void selection_sort_std(std::vector<int> &input, bool dir = 0){
    // iterate through vector, select the lowest in the unsorted sub vector and insert the lowest/ highest in the right place.
    print(input);
    for(int i = 0; i < input.size(); ++i){
        int min_found = std::numeric_limits<int>::max() ;
        int swap_index = i; 
        bool sorted = 1;
        for(int j = i+1; j < input.size(); ++j){
            if(input.at(j) < min_found){
                min_found = input.at(j);
                swap_index = j;
                sorted = 0;
            }
        }
        if(swap_index != i){
            swap(input, i, swap_index);
        }
        if(sorted) {
            std::cout << "sorted" ;
            break;
        }
        print(input);
    }
}

// sort in place
void selection_sort_mod1(std::vector<int> &input, bool dir = 0){
    // iterate through vector, select the lowest in the unsorted sub vector and insert the lowest/ highest in the right place.
    print(input);
    for(int i = 0; i < input.size(); ++i){
        // int min_found = std::numeric_limits<int>::max() ;
        int swap_index = i; 
        // bool sorted = 1;
        for(int j = i+1; j < input.size(); ++j){
            if(input.at(j) < input.at(i)){
                // min_found = input.at(j);
                swap_index = j;
                // sorted = 0;
            }
        }
        if(swap_index != i){
            swap(input, i, swap_index);
        }
        // if(sorted) {
        //     std::cout << "sorted" ;
        //     break;
        // }
        print(input);
    }
}


//sort in place
void insertion_sort_std(std::vector<int>& input){
    // iterate through vector, find position of current element in the sorted subarray, and insert where required.
    print(input);
    if(input.size() > 1){    
        for(int i = 1; i < input.size(); ++i){
            int swap_index = i; 
            // bool sorted = 1;
            for(int j = i-1; j >= 0; --j){
                if(input.at(j) > input.at(swap_index)){
                    swap(input, j, swap_index);
                    swap_index --;
                }
            }
            print(input);
        }
    }
}


//sort in place
void bubble_sort_mod1(std::vector<int>& in){
    int counter = 0;
    for(int i = 0; i< in.size(); ++i){
        int swapped = 0;
        for(int j = 1; j < in.size()-counter; j++){
            if(in.at(j) < in.at(j-1)){
                swap(in, j, j-1);
                swapped = 1;
            }
        }
        counter++;
        print(in);
        if(!swapped)break;
    }
}

// what is this??
void bubble_sort_mod2(std::vector<int>& in){
    for(int i = 0; i< in.size(); ++i){
        for(int j = i+1; j < in.size(); ++j){
            if(in.at(i) < in.at(j)){
                swap(in, i, j);
            }
        }
        print(in);
    }
}


void bubble_sort_std(std::vector<int>& in){
    print(in);
    if(in.size()>1){
        int i = 0;
        bool swapped = 0;
        while(1){
            if(in.at(i) > in.at(i+1)){
                swap(in, i, i+1);
                swapped = 1;
            }
            if(i < in.size()-2){
                ++i;       
            }
            else{
                print(in);
                if(!swapped){
                    break;
                }
                i = 0;
                swapped = 0;
            }
        }
    }   
}


// merge sort
void merge_sort(std::vector<int>& in, std::vector<int>& out){
    // divide the input in 2 vectors, sort each vector, merge in sorted manner
    
    // std::cout << "in ";
    // print(in);
    int halfway = in.size()/2;
    std::vector<int> a(in.begin(), in.begin()+halfway);
    std::vector<int> b(in.begin()+halfway, in.end());
    std::vector<int> a_out, b_out;
    // print(a);
    // print(b);
    if(a.size() > 1) merge_sort(a, a_out);
    else a_out = a;
    if(b.size() > 1) merge_sort(b, b_out);
    else b_out = b;
    
    //merge(a, b);
    int a_it = 0;
    int b_it = 0;
    while(a_it < a_out.size() && b_it < b_out.size()){
        int p = a_out.at(a_it);
        int q = b_out.at(b_it);
        // std::cout<< "p and q: " << p << " " << q << std::endl;
        if(p < q){
            out.push_back(p);
            a_it++;
        }
        else if(p == q){
            out.push_back(p);
            out.push_back(q);
            a_it++;
            b_it++;
        }
        else{
            out.push_back(q);
            b_it++;
        }
    }
    while(a_it < a_out.size()){
        out.push_back(a_out.at(a_it));
        a_it++;
    }
    while(b_it < b_out.size()){
        out.push_back(b_out.at(b_it));
        b_it++;
    }  
    // std::cout << "out ";
    print(out);    
}

void quick_sort(std::vector<int>& in, std::vector<int>& out){
    // pick pivot, place in the right position in the array, repeat for divided subarrays
    print(in);
    // pick pivot
    int pivot = in.back();
    std::vector<int> less, more, equal, less_out, more_out;
    for(auto& i : in){
        if(i < pivot) less.push_back(i);
        else if(i > pivot) more.push_back(i);
        else equal.push_back(i);
    }
    std::cout << "------- "<< std::endl;
    // print(less);
    // print(equal);
    // print(more);
    
    if(less.size() > 1) quick_sort(less, less_out);
    else less_out = less;
    if(more.size() > 1) quick_sort(more, more_out);
    else more_out = more;
    // in.clear();
    for(auto &i : less_out){
        out.push_back(i);
    }
    for(auto &i : equal){
        out.push_back(i);
    }
    for(auto &i : more_out){
        out.push_back(i);
    }
    // std::cout<< "out " ;
    // print(out);
    
}


int main() {
    
    std::vector<int> in = test_vector(10);
    std::vector<int> out;
    quick_sort(in, out);
    
    
}