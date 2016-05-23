#ifndef OPENRAVE_FCL_STATISTICS
#define OPENRAVE_FCL_STATISTICS


#ifdef FCLUSESTATISTICS

#include "plugindefs.h"
#include <sstream>

namespace fclrave {

class FCLStatistics;

static std::vector<boost::weak_ptr<FCLStatistics> > globalStatistics;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> time_point;
typedef std::chrono::duration<double> duration;

class FCLStatistics {
public:
    FCLStatistics(std::string const& key, int id) {
        name = str(boost::format("%d;%s")%id%key);
        RAVELOG_WARN_FORMAT("FCL STATISTICS %s", name);
        currentTimings.reserve(64); // so that we don't allocate during the timing
    }

    ~FCLStatistics() {
        //Display();
    }

    void DisplayAll() {
        FOREACH(itstat, globalStatistics) {
            boost::shared_ptr<FCLStatistics> pstat = itstat->lock();
            if( pstat ) {
                pstat->Display();
            }
        }
    }

    void DisplaySingle(const std::string& label, std::vector<time_point> timings) {
        std::stringstream ss;
        std::vector<time_point>::iterator it = timings.begin();
        time_point t = *it;
        while(++it != timings.end()) {
            ss << ";" << (*it - t).count();
            t = *it;
        }
        RAVELOG_WARN_FORMAT("FCL STATISTICS;%s%s", label % ss.str());
    }

    void Display() {
        FOREACH(ittiming, timings) {
            std::vector<double> durations;
            durations.reserve(ittiming->second.size());
            FOREACH(itd, ittiming->second) {
                durations.push_back(itd->count());
            }
            double max = *std::max_element(durations.begin(), durations.end());
            const double numCells = 20, cellSize = max/numCells;
            std::vector<int> histograph(0, numCells);
            double mean = 0;
            FOREACH(itd, durations) {
                mean += *itd;
                //TODO:correct the formula which seems to be wrong
                //histograph.at(static_cast<int>(std::floor(numCells * *itd / max)))++;
            }
            mean /= static_cast<double>(durations.size());
            RAVELOG_WARN_FORMAT("%s %s %d %Lf\n", name%ittiming->first%durations.size()%mean);
        }
    }

    void StartManualTiming(std::string const& label) {
        currentTimingLabel = str(boost::format("%s;%s")%name%label);
        currentTimings.resize(0);
        currentTimings.push_back(std::chrono::high_resolution_clock::now());
    }

    void StopManualTiming() {
        currentTimings.push_back(std::chrono::high_resolution_clock::now());
        duration timing = *currentTimings.begin() - *(currentTimings.end()-1);
        timings[currentTimingLabel].push_back(timing);
        DisplaySingle(currentTimingLabel, currentTimings);
    }

    void AddTimepoint() {
        currentTimings.push_back(std::chrono::high_resolution_clock::now());
    }

    struct Timing {
        Timing(FCLStatistics& statistics) : _statistics(statistics) {
        }
        ~Timing() {
            _statistics.StopManualTiming();
        }
private:
        FCLStatistics& _statistics;
    };

    Timing StartTiming(std::string const& label) {
        StartManualTiming(label);
        return Timing(*this);
    }

    std::string name;
    std::string currentTimingLabel;
    std::vector<time_point> currentTimings;
    std::map< std::string, std::vector<duration> > timings;
};

typedef boost::shared_ptr<FCLStatistics> FCLStatisticsPtr;

#define SETUP_STATISTICS(statistics, userdatakey, id) \
    statistics = boost::make_shared<FCLStatistics>(userdatakey, id); \
    globalStatistics.push_back(boost::weak_ptr<FCLStatistics>(statistics))

#define START_TIMING(statistics, label) FCLStatistics::Timing t = statistics->StartTiming(label)

#define ADD_TIMING(statistics) statistics->AddTimepoint()

#define DISPLAY(statistics) statistics->DisplayAll()

} // fclrave

#else // FCLUSESTATISTICS is not defined

namespace fclrave {

class FCLStatistics {
};

#define SETUP_STATISTICS(statistics, userdatakey, id) do {} while(false)
#define START_TIMING(statistics, label) do {} while(false)
#define ADD_TIMING(statistics) do {} while(false)
#define DISPLAY(statistics) do {} while(false)

}
#endif



#endif
