Given an array of size n and an integer k, return the count of distinct numbers in all windows of size k.

Input: arr[] = {1, 2, 1, 3, 4, 2, 3};
       k = 4
Output: 3 4 4 3

Explanation:
First window is {1, 2, 1, 3}, count of distinct numbers is 3
start = 0	end = 3 a = 0  counter = 1  

Second window is {2, 1, 3, 4} count of distinct numbers is 4
Third window is {1, 3, 4, 2} count of distinct numbers is 4
Fourth window is {3, 4, 2, 3} count of distinct numbers is 3

// loop through the outer array, 
// loop through window
// increment count if another element found

int start = 0;
int end = k-1;
vector<int> out{};

while(end != arr.size()-1){
	int counter = start+1;
  int distinct = k;
  int ele = start // start of array
  while(ele!=end){
    for(int i = ele+1 ; i< k; i++){
      if(arr.at(ele) == arr.at(i)) distinct--;
    }
    ele++  
  }
  out.push_back(distinct);
	start++;
  end++;
}


int ele = start // start of array
for(int i = ele+1 ; i< k; i++){
	if(arr.at(ele) == arr.at(i)) distinct--;
}
ele++


