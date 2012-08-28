// next_combination.h. This code is in the public domain. Created by Hannu Helminen.

#include <algorithm>

template<typename Value, typename Iterator>
void disjoint_rotate(Iterator begin1, Iterator end1, size_t size1,
                     Iterator begin2, Iterator end2, size_t size2,
                     Value *type) {
    const size_t total = size1 + size2;
    size_t gcd = total;
    for (size_t div = size1; div != 0; ) {
        gcd %= div;
        std::swap(gcd, div);
    }
    const size_t skip = total / gcd - 1;
    for (size_t i = 0; i < gcd; ++i) {
        Iterator curr((i < size1) ? begin1 + i : begin2 + (i - size1));
        size_t ctr = i;
        const Value v(*curr);
        for (size_t j = 0; j < skip; ++j) {
            ctr = (ctr + size1) % total;
            Iterator next((ctr < size1) ? begin1 + ctr : begin2 + (ctr - size1));
            *curr = *next;
            curr = next;
        }
        *curr = v;
    }
}

template<typename Iterator>
bool next_combination(Iterator begin, Iterator mid, Iterator end) {
    if (begin == mid || mid == end) {
        return false;
    }
    // Starting from mid backwards, find first char that is
    // less than last char. Call it head_pos. This is the one
    // which we will increment (swap)
    Iterator tail_pos(end);
    --tail_pos;
    Iterator head_pos(mid);
    --head_pos;
    size_t head_len = 1;
    while (head_pos != begin && !(*head_pos < *tail_pos)) {
        --head_pos;
        ++head_len;
    }
    if (head_pos == begin && !(*head_pos < *tail_pos)) {
        // Last combination. We know that the smallest elements are
        // in tail (in order) and largest elements are in head (also
        // in order). rotate everything back into order and return false.
        std::rotate(begin, mid, end);
        return false;
    }
    // Now decrement tail_pos as long as it is larger than *head_pos.
    // This way we'll find the two positions to swap.
    size_t tail_len = 1;
    while (tail_pos > mid) {
        --tail_pos;
        ++tail_len;
        if (!(*tail_pos > *head_pos)) {
            ++tail_pos;
            --tail_len;
            break;
        }
    }
    // Now we have head_pos and tail_pos. Lets call
    //  - 'h': the element at head_pos
    //  - 'H': elements head_pos+1 .. mid
    //  - 't': the element at tail_pos
    //  - 'T': elements tail_pos+1 .. end
    // First, we know that the following is a non-decreasing sequence:
    // h <= t <= T <= H. (The sequences are also ordered.)
    // We should now re-order the elements so that h and t swap places,
    // and the rest of T and H are arranged in the the remaining places
    // in increasing order. Let's call the gap between mid and tail_pos ';'.
    // Using this notation, the reorder is  h H ; t T -> t TH ; h TH
    // where the repeated TH means that the concatenation of sequences TH is
    // split into two pieces to take place of the former members.

    // Some special cases to avoid unnecessary work.
    // If head_len == 1 (H == ''), a simple swap of h and t will always suffice.
    // Proof: h ; t T -> t ; h T, so T will occupy the same location as before.

    // Same is true if tail_len == 1 (T == ''). Proof: h H ; t -> t H ; h,
    // so this time H will stay in place and swap will suffice.
    if (head_len == 1 || tail_len == 1) {
        std::iter_swap(head_pos, tail_pos);
        return true;
    }
    // If head_len == tail_len, this operation reduces to a swap_ranges.
    // Proof: since len(H) == len(T), the swap is h H ; t T -> t T ; h H
    if (head_len == tail_len) {
        std::swap_ranges(head_pos, mid, tail_pos);
        return true;
    }
    // Finally we have to do the full reorder. Simply swap h and t, then
    // do what std::rotate would do with the sequence H T with the
    // constraint that the elements are not stored in consecutive locations.
    std::iter_swap(head_pos, tail_pos);
    disjoint_rotate(head_pos + 1, mid, head_len - 1, tail_pos + 1, end, tail_len - 1, &*head_pos);
    return true;
}
