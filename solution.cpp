#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>

using namespace std;

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    
    int T;
    cin >> T;
    
    while (T--) {
        int n;
        cin >> n;
        vector<long long> a(n);
        
        for (int i = 0; i < n; i++) {
            cin >> a[i];
        }
        
        // Calculate the sum of all elements
        long long total_sum = 0;
        for (int i = 0; i < n; i++) {
            total_sum += a[i];
        }
        
        long long min_s = LLONG_MAX;
        
        // Key insight: We can consolidate all values into the first k positions
        // for different values of k to minimize S
        // Since operations only allow moving values leftward (j > i),
        // the optimal strategy is to distribute the total sum among
        // the leftmost k positions as evenly as possible
        for (int k = 1; k <= n; k++) {
            // Distribute total_sum among first k positions as evenly as possible
            // in non-increasing order to satisfy the constraint
            vector<long long> optimal(n, 0);
            
            long long base = total_sum / k;
            long long remainder = total_sum % k;
            
            // Fill the first k positions
            for (int i = 0; i < k; i++) {
                optimal[i] = base;
                if (i < remainder) {
                    optimal[i]++;
                }
            }
            
            // Sort the first k positions in non-increasing order
            sort(optimal.begin(), optimal.begin() + k, greater<long long>());
            
            // Calculate S = Σ(min(a₁,...,aᵢ) + max(a₁,...,aᵢ)) for this configuration
            long long s = 0;
            for (int i = 0; i < n; i++) {
                long long min_val, max_val;
                
                if (i < k) {
                    // Find min and max in first i+1 positions
                    min_val = *min_element(optimal.begin(), optimal.begin() + i + 1);
                    max_val = *max_element(optimal.begin(), optimal.begin() + i + 1);
                } else {
                    // All positions from k onwards are 0, so min is 0
                    min_val = 0;
                    max_val = optimal[0]; // max is always the first element
                }
                
                s += min_val + max_val;
            }
            
            min_s = min(min_s, s);
        }
        
        cout << min_s << "\n";
    }
    
    return 0;
}