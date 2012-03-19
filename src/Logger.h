#ifndef __BOOST_BASE_LOGGER_H
#define __BOOST_BASE_LOGGER_H

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <ctime>

#include <string>
#include <queue>

class Logger;

/*
 * simple container class for log entries 
 */
struct LogEntry
{
	std::string				 file;
	int						 line;
	std::string				 component;
	boost::posix_time::ptime ts;
	std::string				 message;

	LogEntry() {}

	LogEntry(const std::string& f, 
			 int l, 
			 const std::string& c, 
			 boost::posix_time::ptime t, 
			 const std::string&  m) :
	file(f), line(l), component(c), ts(t), message(m) {}
};


template<class T>
class Singleton : private boost::noncopyable
{
 public:
    static T& instance()
    {
        boost::call_once(&createInstance, s_once); return *s_instance;
    }
    virtual ~Singleton(){}

 protected:
    Singleton()  { }
    static void createInstance()  {  s_instance.reset(new T());  }

 private:
    static boost::once_flag s_once;
    static boost::scoped_ptr<T> s_instance;
};

/*
 * An interface for formatting output in a logger
 */
class LoggerFormatter :  boost::noncopyable
{
	public:
		virtual std::string  format(const Logger& logger, 
									 const std::string& delim, 
									 const LogEntry& logMessage) = 0;
};

/*
 * default implementation of a formatter. Add any fields you feel like worth outputing 
 * in a constructor and change format function accordingly.
 */
class DefaultLoggerFormatter : public LoggerFormatter
{
	public:
		virtual std::string  format(const Logger& logger, 
									 const std::string& delim, 
									 const LogEntry& logMessage);

};

/*
 * concurrent queue implementation
*/
template<typename Data>
class concurrent_queue
{
private:
    std::queue<Data>			m_queue;
    mutable boost::mutex		m_mutex;
    boost::condition_variable	m_condition_variable;
	bool						m_shutdown;
public:
	
	concurrent_queue() : m_shutdown(false) {}

    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(m_mutex);
        m_queue.push(data);
        lock.unlock();
        m_condition_variable.notify_one();
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(m_mutex);
        return m_queue.empty();
    }

	void wait_and_pop(std::vector<Data>& values)
    {
        boost::mutex::scoped_lock lock(m_mutex);
        while(m_queue.empty() && !m_shutdown)
        {
            m_condition_variable.wait(lock);
        }
        
		while(!m_queue.empty())
		{
	        values.push_back(m_queue.front());
		    m_queue.pop();
		}
    }

	void shutdown()
	{
		m_shutdown = true;
		//unblock waiting thread
        m_condition_variable.notify_one();
	}
};

/*
 *  Logger implementation sigleton. You should use something like this to start using logger
 *  Please note you may lose some messages if you try to log anything after the shutdown has already initiated
 * 	Logger::setOutputFormatter(boost::shared_ptr<LoggerFormatter>(new DefaultLoggerFormatter()));
 *	Logger::info("%s\n", "this is just a log message");
 *
 */

class Logger :  Singleton<Logger>
{
	public:
		typedef enum {
			LOG_LEVEL_FATAL = 0,
			LOG_LEVEL_ERROR,
			LOG_LEVEL_WARN,
			LOG_LEVEL_INFO,
			LOG_LEVEL_DEBUG
		} LOGLEVEL;

        Logger();
		virtual ~Logger();

	    /**
	     * Sets the logging level. All messages with lower priority will be ignored.
	     */
	    static void setLevel(LOGLEVEL level) { instance().m_logLevel = level; }

	    /**
	     * Gets the defaul logging level.
	     */
	    static LOGLEVEL getLevel() { return instance().m_logLevel; }

	    /**
	     * Sets the log file name
	     */
		static void setFileName(const std::string& name) { instance()._setFileName(name); }

	    /**
	     * Sets the log file name
	     */
		static void setRotationSize(long sz) { instance()._setRotationSize(sz); }

	    /**
	     * Log a message to a log file. The method returns immediately.
		 * A timestamp of the log message is taken at a time when log method is being called
		 * An actual logging will happen in a separate thread.
	     */
	    static int log(LOGLEVEL level, 
						const char* file, 
						int line, 
						const boost::posix_time::ptime& ts, 
						const char* component,
						const char* fmt, ...);
	
		/*
		 * Set's field delimiter, it's a blank space by default
		 */
		static void setOutputFormatter(boost::shared_ptr<LoggerFormatter>& formatter)
		{
			instance().m_formatter = formatter;
		}

		/*
		 * Set's field delimiter, it's a blank space by default
		 */

	#define LOG(NAME,LEVEL) \
	static int NAME(const char* file, int line, const boost::posix_time::ptime& ts, const char* component, const char* fmt, ...) \
		{ \
			va_list ap; \
			va_start(ap, fmt); \
			int ret = instance()._log(LEVEL, file, line, ts, component, fmt, ap); \
			va_end(ap); \
			return ret; \
		}
	
	    LOG(fatal, LOG_LEVEL_FATAL)
	    LOG(error, LOG_LEVEL_ERROR)
	    LOG(warning, LOG_LEVEL_WARN)
	    LOG(info, LOG_LEVEL_INFO)
	    LOG(debug, LOG_LEVEL_DEBUG)

	private:
		
        //		Logger();
		//virtual ~Logger();
	
        /*         static Logger& instance() 
		{       
			static Logger logger;
			return logger;
            }*/

		void _setFileName(const std::string& name);
		void _setRotationSize(long sz);

	    int _log(LOGLEVEL level, 
				 const char* file, 
				 int line, 
				 const boost::posix_time::ptime& ts, 
				 const char* component,
				 const char* fmt, 
				 va_list arglist);

		void addToQueue(const std::string& message);

		void startQueue();
		void stopQueue();
		void processQueue();

		void rotateFile();
	private:

	    FILE*								m_stream;
	    LOGLEVEL							m_logLevel;
		std::string							m_delim;

		boost::shared_ptr<LoggerFormatter>	m_formatter;

	    boost::thread						m_QueueThread;
		volatile bool						m_bRunning;
		concurrent_queue<LogEntry>			m_queue;

		std::string							m_archivefmt;
		std::string							m_filepath;
		long								m_rotatepos;

		mutable boost::mutex				m_fop_mutex;
};

template<class T> boost::once_flag Singleton<T>::s_once = BOOST_ONCE_INIT;
template<class T> boost::scoped_ptr<T> Singleton<T>::s_instance;

typedef boost::date_time::microsec_clock<boost::posix_time::ptime> utc_mcs_time;
typedef boost::date_time::second_clock<boost::posix_time::ptime> utc_sec_time;

#define LOG_FATAL(component, ...)	Logger::fatal	(__FILE__,__LINE__, utc_mcs_time::universal_time(), component, __VA_ARGS__)
#define LOG_ERROR(component,...)	Logger::error	(__FILE__,__LINE__, utc_mcs_time::universal_time(), component, __VA_ARGS__)
#define LOG_WARN(component,...)		Logger::warning	(__FILE__,__LINE__, utc_mcs_time::universal_time(), component,__VA_ARGS__)
#define LOG_INFO(component,...)		Logger::info	(__FILE__,__LINE__, utc_mcs_time::universal_time(), component,__VA_ARGS__)
#define LOG_DEBUG(component,...)	Logger::debug	(__FILE__,__LINE__, utc_mcs_time::universal_time(), component, __VA_ARGS__)

#endif
