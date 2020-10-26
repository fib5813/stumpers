/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    
    ListNode* find_it2_and_tail(ListNode*& it){
        // _ _ prev->it _ _ _ tail->it2
        ListNode* tail = nullptr; 
        // ListNode* it2 = nullptr;
        bool needs_sorting = false;
        while(it!= nullptr && it->next!=nullptr){
            if(it->val < it->next->val) it = it->next;
            else {
                needs_sorting = true;
                tail = it;
                
                // it2 = it->next; 
                break;
            }
        }
        std::cout << tail->val << " " <<  " " << endl;
        return tail;
    }
    
    void my_replace(ListNode*& prev, ListNode*& it, ListNode*& tail, ListNode*& it2){
        // _ _ prev->it _ _ _ tail->it2
        // _ _ prev->it2->it _ _ tail ->it2_next
        // cout <<  prev->val << " " << prev->next->val << " "  
        //     << tail->val << " " << tail->next->val << " " << tail->next->next->val << std::endl;
        std::cout<< "prev_val" <<endl;
        tail->next = it2->next;
        it2->next = it;
        prev->next = it2;
        
        // cout <<  prev->val << " " << prev->next->val << " " <<prev->next->next->val << " " 
        //     << tail->val << " " << tail->next->val;
    }
    
    ListNode* insertionSortList(ListNode* head) {
        if(head == nullptr || head->next == nullptr) return head;
        
        // iterate through to find first eement which is lesser than previous element
        auto it = head;
        ListNode* it2 = nullptr;
        ListNode* tail = nullptr;
        tail = find_it2_and_tail(it);
        it2 = tail->next;
        
        // auto it = head;
        // while(it2 != nullptr){
            // head_ _ prev->it _ _ _ tail->it2
            it = head;
            ListNode* prev = head;
            if(it == tail ) std::cout << "stupid";
            while(it!=tail){
                if(it2->val > it->val ){
                    prev = it;
                    it = it->next;
                    std::cout << it->val << " here";
                } 
                // std::cout << std::endl;
                else my_replace(prev, it, tail, it2);
                
            }
            
        // }    
        
        std::cout << tail->val << " " << it2->val << " " << endl;
        
        
        return head;
    }
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    
    void arrange_ele(ListNode*& head, ListNode*& curr, ListNode*&tail){
        ListNode* it = head;
        ListNode* prev = head;
        while(it!=nullptr){
            //prev->it _ _ _ tail->curr
            if(curr->val >= it->val){
                prev = it;
                it = it->next;
            }
            else{
                
                //prev->curr->it _ _ _ tail ->new_curr
                if(it ==  prev) { //replace head by curr element
                    
                    
                    // head = curr;
                    // ListNode* temp = curr->next;
                    // head->next = it;
                    // it->next = temp;
                    // tail = it;
                    // curr = temp;
                }
                else {
                    prev->next = curr;
                    tail = curr->next;
                    curr->next = it;
                    
                }
                cout << head->val << " " << head->next->val << " " << head->next->next->val <<endl;
                break;
            }
        }
        
        // cout << curr->val << it->val;
        // while(curr->val < it->val){
        //     curr = curr->next;
        //     if(it->next!= nullptr){
        //         it = it->next;
        //     }
        // }
        // if(curr->next == nullptr) tail = nullptr;
        // else tail = curr;
        // cout << tail->val << " function" << endl;
    }
    
    ListNode* insertionSortList(ListNode* head) {
        if(head==nullptr || head->next == nullptr)return head;
        // head _ _ curr->it _ _ _ tail->it2 _ _ _
        ListNode* tail = head;
        ListNode* it2 = nullptr;
        ListNode* curr = head->next;
        // ListNode* it = head->next;
       
        
        arrange_ele(head, curr, tail);  //get first pair of unsorted elements
        arrange_ele(head, curr, tail);  //get first pair of unsorted elements

        // it2 = tail->next;
        // std::cout << curr->val << " " << it->val << " " << tail->val << " " << it2->val <<endl;
        
        if(tail == nullptr)return head; // input is already sorted/
        
        
        
        
        return head;
    }
};



// finallyy!!


/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    void replace_head(ListNode*& head, ListNode*& rem){
        rem->next = head;
        head = rem;
        // cout << "replaced head " << head->val << " " << head->next->val << endl;
    }
    
    void insert_rem(ListNode*& it, ListNode*& rem){
        rem->next = it->next;
        it->next = rem;
        // cout<< "inserted element " << it->val << " " << it->next->val << " " << it->next->next->val <<endl;
    }
    
    void arrange_ele(ListNode*& head, ListNode*&tail){
        // update the head, curr, tail
        ListNode* it = head;
            
        // if elements are sorted, just increment tail.
        if(tail->next==nullptr) return;
        
        if(tail->next->val >= tail->val){
            tail = tail->next;
            // std::cout << "if condition- " << head->val << " " << tail->val <<endl;
            // this gives the sorted list.
        }
            
        //else, something needs to be changed.
        // 1. element to be changed is rem = tail-> next (as long as it is not nullptr)
        // 2. remove this element from the list.
        // 3. iterate through the list to find the right place such that prev->it.val < rem.val < tail.val 
        //      1. if(it == prev) head needs to be replaced
        //      2. else if(it!=tail) element needs to be inserted between prev and it
        else{
            // cout << "else condition - " << head->val << " " << tail-> val<< endl;
            ListNode* rem = tail->next;
            tail->next = rem->next;
            rem->next = nullptr;
            int val = (head->next == nullptr) ? -1 : head->next->val;
    
            // cout<< "after removal - " << head->val << " " << rem->val << " " << (head->next == nullptr) << " " << val <<endl;
            
            
            if(rem->val < head->val) {
                replace_head(head, rem);
                return;
            }
            
            ListNode* it = head;
            while(it!= tail){ // iterate through tail
                // cout << "while loop " << rem->val << " " << it->next->val <<endl ;

                if(rem->val < it->next->val ){ //  it->rem->(it->next){
                    // cout<< "here " << rem->val << " " << it->next->val << endl;
                    insert_rem(it, rem);
                    return;
                }
                it = it->next;
            }
        }
            
        
    }
    
    ListNode* insertionSortList(ListNode* head) {
        if(head==nullptr || head->next == nullptr)return head; // if list is less than 2, return same list
        
        ListNode* tail = head;
        while(tail->next!=nullptr){
            arrange_ele(head, tail);  //get first pair of unsorted elements
        }  
        if(tail == nullptr)return head; // input is already sorted/        
        return head;
    }
};