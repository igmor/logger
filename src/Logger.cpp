#include "ssBase_Boost.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/filesystem/convenience.hpp"

#include "Logger.h"
#include <sstream>

static const uint32 MAX_SNPRINTF_BUF_SIZE = 2048;

//MAX_PATH has too many issues, we just need reasonable buffer size
static const uint32 MAX_PATH_LENGTH = 2048;

//windows renamed posix string functions to an unsafe version _s*
#ifdef _WIN32
#define snprintf _snprintf
#endif

std::string DefaultLoggerFormatter::format( const Logger& logger, 
											const std::string& delim, 
											const LogEntry& logEntry)
{
	std::ostringstream s(std::ostringstream::out);
	s << delim 
		<< Logger::getLevelToken(logger.getLevel()) << delim 
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
}

Logger::~Logger()
{
	stopQueue();
	if (m_stream!=NULL)
		fclose(m_stream);
}

std::string Logger::getLevelToken(Logger::LOGLEVEL level)
{
	switch (level)
	{
		case LOG_LEVEL_SILENT: return "SILENT"; break;
		case LOG_LEVEL_FATAL: return "FATAL"; break;
		case LOG_LEVEL_ERROR: return "ERROR"; break;
		case LOG_LEVEL_WARN: return "WARN"; break;
		case LOG_LEVEL_INFO: return "INFO"; break;
		case LOG_LEVEL_DEBUG: return "DEBUG"; break;
		case LOG_LEVEL_VERBOSE: return "VERBOSE"; break;
		default:
			return "UNDEFINED_LOG_LEVEL";
	}	
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

	if (level < m_logLevel) 
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
	m_queue.shutdown();
	m_QueueThread.join();
	//clear the queue
	std::vector<LogEntry> logEntries;
	m_queue.pop(logEntries);

	format(logEntries);
}

void Logger::rotateFile()
{            
	if (!m_stream)
		return;

	// check if we need to rotate 
	if (m_rotatepos > 0)
	{
		// close, rename, reopen-truncated..
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

void Logger::format(const std::vector<LogEntry>& entries)
{
	for (unsigned int i = 0; i < entries.size(); i++)
		if (m_formatter)
		{
			std::string message = m_formatter->format(*this, m_delim, entries[i]);
			if (m_stream)
			{
				fprintf(m_stream, "%s\n", message.c_str());
				fflush(m_stream);

				boost::mutex::scoped_lock lock(m_fop_mutex);

				long  cur = ftell(m_stream);
				if (cur >= m_rotatepos)
					rotateFile();
			}
		}
}

void Logger::processQueue()
{
	while (m_bRunning)
	{
		std::vector<LogEntry> logEntries;
		m_queue.wait_and_pop(logEntries);

		format (logEntries);
	}
}

