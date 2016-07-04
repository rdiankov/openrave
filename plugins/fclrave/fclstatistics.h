#ifndef OPENRAVE_FCL_STATISTICS
#define OPENRAVE_FCL_STATISTICS


#ifdef FCLUSESTATISTICS

#include "plugindefs.h"
#include <sys/stat.h>
#include <sstream>
#include <fstream>


namespace fclrave {

class FCLStatistics;

static std::vector<boost::weak_ptr<FCLStatistics> > globalStatistics;
static EnvironmentMutex log_out_mutex;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> time_point;
typedef std::chrono::duration<double> duration;

class FCLStatistics {
public:
    FCLStatistics(std::string const& key, int id) {
        name = str(boost::format("%d;%s")%id%key);
        RAVELOG_WARN_FORMAT("FCL STATISTICS %s", name);
        currentTimings.reserve(64); // so that we don't allocate during the timing

        const size_t bufferSize = 600; // ~200 objects * 2 int per objects * 1.5 margin
        envCaptureLogFile = str(boost::format("envCapture-%d-%s.log")%id%key);
        struct stat buffer;
        if( stat(envCaptureLogFile.c_str(), &buffer) == 0 ) {
            fstream f(envCaptureLogFile, std::fstream::trunc);
            f.close();
        }
        envCaptureCount = 0;
        // don't want to allocate during the main loop
        _tmpbuffer.reserve(bufferSize);
        for(int i = 0; i < maxEnvCaptureCount; ++i) {
            vEnvCapture[i].reserve(bufferSize);
        }
    }

    ~FCLStatistics() {
        if( envCaptureCount > 0 ) {
            FlushEnvCaptureBuffer();
        }
#ifndef FCL_STATISTICS_DISPLAY_CONTINUOUSLY
        Display();
#endif
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
        EnvironmentMutex::scoped_lock lock(log_out_mutex);
        std::fstream f("fclstatistics.log", std::fstream::out | std::fstream::app);
        FOREACH(ittiming, timings) {
            f << ittiming->first;
            size_t maxTimingCount = 0;
            FOREACH(ittimingvector, ittiming->second) {
                f << ";";
                maxTimingCount = std::max(maxTimingCount, ittimingvector->size() - 1);
                std::vector<time_point>::iterator it = ittimingvector->begin();
                time_point t = *it;
                while(++it != ittimingvector->end()) {
                    f << "|" << (*it - t).count();
                    t = *it;
                }
            }
            f << ";" << maxTimingCount << std::endl;
        }
    }

    void StartManualTiming(std::string const& label) {
        currentTimingLabel = str(boost::format("%s;%s")%name%label);
        currentTimings.resize(0);
        currentTimings.push_back(std::chrono::high_resolution_clock::now());
    }

    void StopManualTiming() {
        currentTimings.push_back(std::chrono::high_resolution_clock::now());
        timings[currentTimingLabel].push_back(currentTimings);
#ifdef FCL_STATISTICS_DISPLAY_CONTINUOUSLY
        DisplaySingle(currentTimingLabel, currentTimings);
#endif
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

    void FlushEnvCaptureBuffer() {
        RAVELOG_DEBUG_FORMAT("Flushing env capture buffer in file %s", envCaptureLogFile);
        std::fstream f(envCaptureLogFile, std::fstream::out | std::fstream::app);
        for(int i = 0; i < envCaptureCount; ++i) {
            if( vEnvCapture[i].size() == 0 ) {
                continue;
            }
            f << "{";
            std::copy(vEnvCapture[i].begin(), vEnvCapture[i].end() - 1, std::ostream_iterator<string>(f, ", "));
            f << *--vEnvCapture[i].end() << "}" << std::endl;
        }
        f.close();
        envCaptureCount = 0;
    }

    void NotifyGeometryGroupChanged(const std::string& groupname) {
        _tmpbuffer.resize(0);
        _tmpbuffer.push_back("\"GeometryGroupChanged\" : \""+ groupname + "\"");
        if( envCaptureCount == maxEnvCaptureCount ) {
            // empty the vEnvCapture buffer when it is full
            FlushEnvCaptureBuffer();
            envCaptureCount = 0;
        }
        vEnvCapture[envCaptureCount].swap(_tmpbuffer);
        envCaptureCount++;
    }

    void CaptureEnvState(const std::set<KinBodyConstPtr>& envbodies, KinBodyConstPtr pbody, int linkindex) {
        RAVELOG_VERBOSE_FORMAT("Capturing env (count = %d, file = %s)", envCaptureCount%envCaptureLogFile);
        _tmpbuffer.resize(0);
        _tmpbuffer.push_back(str(boost::format("\"CollisionQuery\" : { \"Body\" : \"%s\", \"Link\" : %d } ")%pbody->GetName()%linkindex));
        FOREACH(itbody, envbodies) {
            OPENRAVE_ASSERT_OP( (*itbody)->GetLinks().size(), <=, 8*sizeof(int));
            int enabledLinksMask = 0;
            FOREACH(itlink, (*itbody)->GetLinks()) {
                if( (*itlink)->IsEnabled() ) {
                    enabledLinksMask |= 1 << (*itlink)->GetIndex();
                }
            }
            _tmpbuffer.push_back(str(boost::format("\"%s\" : %d")%(*itbody)->GetName()%enabledLinksMask));
        }
        if( envCaptureCount == maxEnvCaptureCount ) {
            // empty the vEnvCapture buffer when it is full
            FlushEnvCaptureBuffer();
            envCaptureCount = 0;
        }
        vEnvCapture[envCaptureCount].swap(_tmpbuffer);
        envCaptureCount++;
    }

    std::string name;
    std::string currentTimingLabel;
    std::vector<time_point> currentTimings;
    std::map< std::string, std::vector< std::vector<time_point> > > timings;

    std::vector<string> _tmpbuffer;
    static const size_t maxEnvCaptureCount = 1000;
    size_t envCaptureCount;
    std::vector<string> vEnvCapture[maxEnvCaptureCount];
    std::string envCaptureLogFile;
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
