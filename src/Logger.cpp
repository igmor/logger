#include "ssBase_Boost.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/filesystem/convenience.hpp"

#include "Logger.h"
#include <sstream>

using namespace std;

const uint32 MAX_SNPRINTF_BUF_SIZE = 2048;

//MAX_PATH has too many issues, we just need reasonable buffer size
const uint32 MAX_PATH_LENGTH = 2048;

//windows renamed posix string functions to an unsafe version _s*
#ifdef _WIN32
#define snprintf _snprintf
#endif

template<class T> boost::once_flag Singleton<T>::s_once = BOOST_ONCE_INIT;
template<class T> boost::scoped_ptr<T> Singleton<T>::s_instance;

std::string getLevelToken(Logger::LOGLEVEL level)
{
	switch (level)
	{
		case 0: return "FATAL"; break;
		case 1: return "ERROR"; break;
		case 2: return "WARN"; break;
		case 3: return "INFO"; break;
		case 4: return "DEBUG"; break;
		default:
			return "UNDEFINED_LOG_LEVEL";
	}	
}
std::string DefaultLoggerFormatter::format( const Logger& logger, 
											const std::string& delim, 
											const LogEntry& logEntry)
{
	ostringstream s(ostringstream::out);
	s << delim 
		<< getLevelToken(logger.getLevel()) << delim 
		<< logEntry.component << delim
		<< logEntry.file << delim
		<< logEntry.line << delim
		<< boost::posix_time::to_simple_string(logEntry.ts) << delim 
		<< logEntry.message << '\n';

	return s.str();
}

//Logger implementation
Logger::Logger() : 
	m_stream(NULL), 
	m_logLevel(LOG_LEVEL_WARN), 
	m_bRunning(false),
	m_delim("`")
{
	startQueue();
}

Logger::~Logger()
{
	stopQueue();
	if (m_stream!=NULL)
		fclose(m_stream);
}

int Logger::log(LOGLEVEL level, 
				const char* file, 
				int line, 
				const boost::posix_time::ptime& ts, 
				const char* component, 
				const char* fmt, ...)
{
	va_list arglist;
	va_start(arglist, fmt);
	int ret = instance()._log(level, file, line, ts, component, fmt,arglist);
	va_end(arglist);
	return ret;
}

void Logger::_setFileName(const std::string& name)
{
	stopQueue();

	if (m_stream != NULL)
		fclose(m_stream);

	if ((m_stream = fopen(name.c_str(), "w+")) == NULL)
		fprintf(stderr, "couldn't open file for logging %s \n", name.c_str());

	m_archivefmt = name;
	m_filepath = name;
	const char* logfname = boost::filesystem::basename(name).c_str();
    const char* logbase = strrchr(logfname,'.');

	if (!logbase)
		m_archivefmt += "-%s";
	else
	{
		int pos = m_archivefmt.find_last_of('.');
		m_archivefmt.replace(pos, 1, "-%s.");
	}

	startQueue();
}

void Logger::_setRotationSize(long sz)
{
	boost::mutex::scoped_lock lock(m_fop_mutex);
	m_rotatepos = sz;
}

int Logger::_log(LOGLEVEL level,  
				 const char* file, 
				 int line, 
				 const boost::posix_time::ptime& ts, 
				 const char* component, 
				 const char* fmt, 
				 va_list arglist)
{
	std::string buf;
	buf.resize(MAX_SNPRINTF_BUF_SIZE);

	if (level > m_logLevel) 
		return -1;
	else
	{
		int ret = vsnprintf(&buf[0], MAX_SNPRINTF_BUF_SIZE - 1, fmt, arglist);
		LogEntry l_entry(file, line, component, ts, buf);
		m_queue.push(l_entry);
		return ret;
	}
}

void Logger::startQueue()	
{
	m_bRunning = true;
	m_QueueThread = boost::thread(&Logger::processQueue, this);
}

void Logger::stopQueue()
{
	m_bRunning = false;
	m_QueueThread.join();
}

void Logger::rotateFile()
{            
	if (!m_stream)
		return;

	// check if we need to rotate 
	if (m_rotatepos > 0)
	{
		// close, rename, reopen-truncated..
		fflush(m_stream);
		fclose(m_stream);
		//construct new timestamp-based filename 
		char newname[MAX_PATH_LENGTH+1];
		{
			std::string date_str = boost::posix_time::to_iso_string(utc_mcs_time::local_time());
			snprintf(newname, sizeof(newname), m_archivefmt.c_str(), date_str.c_str());

		}

		if (0 != rename(m_filepath.c_str(),newname))
			fprintf(stderr, "coundn't rotate files from %s to %s, error = %d", m_filepath.c_str(), newname, errno);
		m_stream = fopen(m_filepath.c_str(), "w+");
	}
}

void Logger::processQueue()
{
	while (m_bRunning)
	{
		LogEntry logEntry;
		while (!m_queue.empty())
		{
			m_queue.wait_and_pop(logEntry);

			if (m_formatter)
			{
				std::string message = m_formatter->format(*this, m_delim, logEntry);
				if (m_stream)
				{
					fprintf(m_stream, "%s\n", message.c_str());

					boost::mutex::scoped_lock lock(m_fop_mutex);

					long  cur = ftell(m_stream);
					if (cur >= m_rotatepos)
						rotateFile();
				}
			}
		}
	}
}

