#ifndef OPENRAVE_FCL_STATISTICS
#define OPENRAVE_FCL_STATISTICS

namespace fclrave {

#ifdef FCLUSESTATISTICS

#include "plugindefs.h"

class FCLStatistics;

static std::vector<boost::weak_ptr<FCLStatistics> > globalStatistics;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> time_point;
typedef std::chrono::duration<double> duration;

class FCLStatistics {
public:
    FCLStatistics(std::string const& key, int id) {
        name = str(boost::format("%d;%s")%id%key);
        RAVELOG_WARN_FORMAT("FCL STATISTICS %s", name);
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

    void DisplaySingle(const std::string& label, duration timing) {
        RAVELOG_WARN_FORMAT("FCL STATISTICS;%s;%Lf", label % timing.count());
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
        currentTimingBegin = std::chrono::high_resolution_clock::now();
    }

    void StopManualTiming() {
        time_point end = std::chrono::high_resolution_clock::now();
        duration timing = end - currentTimingBegin;
        timings[currentTimingLabel].push_back(timing);
        DisplaySingle(currentTimingLabel, timing);
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
    time_point currentTimingBegin;
    std::map< std::string, std::vector<duration> > timings;
};

typedef boost::shared_ptr<FCLStatistics> FCLStatisticsPtr;

#define SETUP_STATISTICS(statistics, userdatakey, id) \
    statistics = boost::make_shared<FCLStatistics>(userdatakey, id); \
    globalStatistics.push_back(boost::weak_ptr<FCLStatistics>(statistics));

#define START_TIMING(statistics, label) FCLStatistics::Timing t = statistics->StartTiming(label);

#define DISPLAY(statistics) statistics->DisplayAll();

#else // FCLUSESTATISTICS is not defined

class FCLStatistics {
};

#define SETUP_STATISTICS(statistics, userdatakey, id)
#define START_TIMING(statistics, label) do {} while(false);
#define DISPLAY(statistics) do {} while(false)

#endif

}

#endif
