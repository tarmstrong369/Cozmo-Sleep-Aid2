import os
import icalendar
import datetime
import dateutil
from dateutil.rrule import rruleset, rrulestr

# CalendarTracker: parses all calendar/*.ics and returns a list of events sorted by starting time
class CalendarTracker:
    # Timespan in days to parse events into the future
    TIMESPAN = 60

    class Event:
        def __init__(self, summary, start_time, end_time):
            self.summary = summary
            self.start_time = start_time
            self.end_time = end_time

        @staticmethod
        def sort_by(event):
            return event.start_time

        def is_happening(self, datetime_utc):
            if self.start_time <= datetime_utc:
                if datetime_utc < self.end_time:
                    # Event is currently happening
                    return 0
                else:
                    # Event has passed
                    return -1
            else:
                # Event hasn't happened yet
                return 1

    def __init__(self):
        now = datetime.datetime.utcnow().replace(tzinfo=dateutil.tz.tzutc())
        before = now - datetime.timedelta(days=1)
        later = now + datetime.timedelta(days=CalendarTracker.TIMESPAN)
        self.events = []
        # Parse through all .ics files in ./calendars/
        folder_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'calendars')) + '/'
        for filename in os.listdir(folder_path):
            if filename.endswith('.ics'):
                with open(folder_path + filename, 'r') as file:
                    cal = icalendar.Calendar.from_ical(file.read())
                    for component in cal.walk():
                        # Parse all events
                        if component.name == "VEVENT":
                            # Extract components
                            summary = component.get('summary')
                            start_time = component.get('dtstart').dt
                            end_time = component.get('dtend').dt
                            # Skip for infinite events or full-day events
                            if start_time is None or not isinstance(start_time, datetime.datetime) \
                                    or end_time is None or not isinstance(end_time, datetime.datetime):
                                continue
                            # Convert to UTC
                            if isinstance(start_time, datetime.datetime):
                                start_time = start_time.astimezone(dateutil.tz.tzutc())
                            if isinstance(end_time, datetime.datetime):
                                end_time = end_time.astimezone(dateutil.tz.tzutc())
                            # Events with repetition rules must be specially processed
                            if component.get('rrule'):
                                recur_rule = component.get('rrule')

                                # If the rule is an UNTIL rule, make sure it terminates in a date
                                recur_rule_until = recur_rule.get('UNTIL')
                                if recur_rule_until is not None:
                                    if isinstance(recur_rule_until[0], datetime.datetime):
                                        recur_rule_until[0] = recur_rule_until[0].astimezone(dateutil.tz.tzutc())
                                    else:
                                        recur_rule_until[0] = end_time.replace(year=recur_rule_until[0].year,
                                                                               month=recur_rule_until[0].month,
                                                                               day=recur_rule_until[0].day)
                                for recur_rule_datetime in self.parse_recur_rule(recur_rule, start_time,
                                                                                 component.get('exdate'),
                                                                                 before, later):
                                    self.events.append(self.Event(summary, recur_rule_datetime,
                                                                  end_time.replace(year=recur_rule_datetime.year,
                                                                                   month=recur_rule_datetime.month,
                                                                                   day=recur_rule_datetime.day)))
                            # Regular events just need to be added
                            else:
                                self.events.append(self.Event(summary, start_time, end_time))
        self.events = sorted(self.events, key=self.Event.sort_by)
        self.has_event_now()

    def has_event_now(self):
        i = 0
        return_val = False
        now = datetime.datetime.utcnow().replace(tzinfo=dateutil.tz.tzutc())
        for event in self.events:
            code = event.is_happening(now)
            if code == -1:
                # Has passed, skip it
                i = i + 1
            elif code == 0:
                # Is happening, return it
                return_val = True
                break
            else:
                # Is not happening, return it
                break
        # Truncate so we don't re-search through things that have passed
        self.events = self.events[i:]
        return return_val

    @staticmethod
    def parse_recur_rule(recur_rule, start_time, exclusions, before, later):
        rules = rruleset()
        # Add the RRULE string with our UTC-adjusted start time
        recur_rule = recur_rule.to_ical().decode('utf-8')
        rules.rrule(rrulestr(recur_rule, dtstart=start_time))
        # Add the exclusion UTC-adjusted datetimes
        if not isinstance(exclusions, list):
            exclusions = [exclusions]
        for exclude_date in exclusions:
            if isinstance(exclude_date, datetime.datetime):
                rules.exdate(exclude_date.astimezone(dateutil.tz.tzutc()))
        # Get all of the intermediate START datetimes generated by this rule for a timespan, making sure to peek back
        datetimes = []
        for dt in rules.between(before, later):
            datetimes.append(dt)
        return datetimes
