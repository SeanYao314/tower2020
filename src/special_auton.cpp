#include <vector>
#include "recording.h"

namespace special_programs {

    using namespace std;
    using namespace recording;
    vector<RecordUnit> ghost_datapoints;
    vector<RecordUnit> phantom_datapoints;
    vector<RecordUnit> joker_datapoints;

    vector<RecordUnit>& ghost() {
        vector<vector<int>> data_points = {
            {1, 2, 3, 4, 5, 6, 7},
            {1, 2, 3, 4, 5, 6, 7},
            {1, 2, 3, 4, 5, 6, 7}
        };

        for (int i=0; i<data_points.size(); ++i) {
            auto dp = data_points[i];
            RecordUnit ru;
            ru.tick = dp[0];
            for (int j=1; j<dp.size(); ++j) {
                ru.units.push_back(dp[j]);
            }
            ghost_datapoints.push_back(ru);
        }

        return ghost_datapoints;
    }

    vector<RecordUnit>& phantom() {
        vector<vector<int>> data_points = {
            {1, 2, 3, 4, 5, 6, 7},
            {1, 2, 3, 4, 5, 6, 7},
            {1, 2, 3, 4, 5, 6, 7}
        };

        for (int i=0; i<data_points.size(); ++i) {
            auto dp = data_points[i];
            RecordUnit ru;
            ru.tick = dp[0];
            for (int j=1; j<dp.size(); ++j) {
                ru.units.push_back(dp[j]);
            }
            phantom_datapoints.push_back(ru);
        }

        return phantom_datapoints;
    }

    vector<RecordUnit>& joker() {
        vector<vector<int>> data_points = {
            {1, 2, 3, 4, 5, 6, 7},
            {1, 2, 3, 4, 5, 6, 7},
            {1, 2, 3, 4, 5, 6, 7}
        };

        for (int i=0; i<data_points.size(); ++i) {
            auto dp = data_points[i];
            RecordUnit ru;
            ru.tick = dp[0];
            for (int j=1; j<dp.size(); ++j) {
                ru.units.push_back(dp[j]);
            }
            joker_datapoints.push_back(ru);
        }

        return joker_datapoints;
    }
}