#include <algorithm>
#include <iostream>
#include <vector>
using namespace std;
class Solution {
public:
    long long minDamage(int power, vector<int>& damage, vector<int>& health) { // 4 байта + 4*2n байта
        int n = damage.size(); // 4 байта
        for (int i = 0; i < n; i++) { // 4 байта
            health[i] = (health[i] + power - 1) / power;
        }

        vector<int> indexes(n); // 4*n байт
        for (int i = 0; i < n; i++) { // 4 байта
            indexes[i] = i;
        }
        auto compare = [&](int i, int j) { // 4 + 4 байта
            return health[i] * damage[j] < health[j] * damage[i];
        };

        sort(indexes.begin(), indexes.end(), compare);

        long long time = 0; // 8 байт
        long long ans = 0; // 8 байт
        for (const auto &i : indexes) {
            time += health[i];
            ans += time * damage[i];
        }
        return ans;
    }
};

int main() {
    Solution solution;
    vector<int> damage = {1, 2, 3, 4};
    vector<int> health = {4, 5, 6, 8};
    int power = 4;

    long long result = solution.minDamage(power, damage, health);
    cout << "Minimum Damage1: " << result << endl; // 39


    vector<int> damage2 = {1,1,1,1};
    vector<int> health2 = {1,2,3,4};
    int power2 = 1;

    long long result2 = solution.minDamage(power2, damage2, health2);
    cout << "Minimum Damage2: " << result2 << endl; // 20


    vector<int> damage3 = {40};
    vector<int> health3 = {59};
    int power3 = 8;

    long long result3 = solution.minDamage(power3, damage3, health3);
    cout << "Minimum Damage3: " << result3 << endl; // 320

    return 0;
}