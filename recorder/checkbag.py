import rosbag
import sys
import numpy
import math
import argparse
from termcolor import colored

verbose = False

def check_headers(headers):
    logs = {}
    log_ids = []
    intervals = []
    tmin = float('inf')
    tmin_world = float('inf')
    tmax = -float('inf')
    last_seq = float('nan')
    for h in headers:
        seq = h.seq
        t = h.stamp.to_sec()
        if t > tmax: tmax = t
        if t < tmin: tmin = t
        if t < tmin_world and t > 1000000000: tmin_world = t
        if math.isnan(last_seq):
            last_seq = seq
            last_t = t
            continue
        delta = (t - last_t) * 1000
        intervals.append(delta)
        if (seq != 0 or last_seq != 0) and seq != last_seq+1:
            logs[t] = (("message drop (" + colored("seq %d->%d", 'red') + " interval %f ms)") % (last_seq, seq, delta))
        elif delta <= 0:
            logs[t] = (("out-of-order (seq %d->%d " + colored("interval %f ms", 'red') + ")") % (last_seq, seq, delta))
        elif delta > 1000:
            logs[t] = (("invalid stamp (seq %d->%d " + colored("stamp %f->%f", 'red') + " interval %f ms)") % (last_seq, seq, last_t, t, delta))
            intervals[-1] = float('nan')
        last_seq = seq
        last_t = t

    if not math.isinf(tmin_world): tmin = tmin_world

    ints = numpy.array(intervals)
    median = numpy.nanmedian(ints)
    ints = numpy.ma.array(ints, mask=(numpy.isnan(ints)))
    print ("%f FPS in %.2f sec. Intervals (ms): median %f, max %f, min %f, std %f" % \
        (1000/median, tmax - tmin + 0.005, median, numpy.max(ints), numpy.min(ints), numpy.std(ints)))

    with numpy.errstate(invalid='ignore'):
        idx = numpy.where(ints > 1.5 * median)[0]
    for i in idx:
        t = headers[i+1].stamp.to_sec()
        if not logs.has_key(t):
            logs[t] = (("large interval (seq %d->%d " + colored("interval %f ms", 'red') + ")") % (headers[i].seq, headers[i+1].seq, ints[i]))

    with numpy.errstate(invalid='ignore'):
        idx = numpy.where(ints < 0.5 * median)[0]
    for i in idx:
        t = headers[i+1].stamp.to_sec()
        if not logs.has_key(t):
            logs[t] = (("small interval (seq %d->%d " + colored("interval %f ms", 'red') + ")") % (headers[i].seq, headers[i+1].seq, ints[i]))

    if args.verbose:
        for t in sorted(logs.keys()): print ("\t" + "%2.0f%%: " % ((t-tmin) / (tmax-tmin) * 100)  + logs[t])
    else:
        ndrop = 0
        norder = 0
        ninvalid = 0
        nsmall = 0
        msg = list('.' * 101)
        for t in logs.keys():
            perc = int((t-tmin) / (tmax-tmin) * 100)
            if perc < 0 or perc > 100: print perc
            if 'drop' in logs[t] or 'large' in logs[t]:
                ndrop += 1
                msg[perc] = 'D'
            elif 'out-of-order' in logs[t]:
                norder += 1
                msg[perc] = 'O'
            elif 'invalid' in logs[t]:
                ninvalid += 1
                msg[perc] = 'I'
            elif 'small' in logs[t]:
                nsmall += 1
                if msg[perc] == '.': msg[perc] = 's'
        ratio = 100. / len(headers)
        ntotal = ndrop + norder + ninvalid + nsmall
        print ('%d drops (%.1f%%); %d out-of-order (%.1f%%); %d invalid stamp (%.1f%%); %d small intervals (%.1f%%); %d total abnormal (%.1f%%)' \
            % (ndrop, ndrop * ratio, norder, norder * ratio, ninvalid, ninvalid * ratio, nsmall, nsmall * ratio, ntotal, ntotal * ratio))
        print (''.join(msg))

def print_highlight(s):
    print (colored(s, 'yellow'))

def check_bag(bag):
    topic_headers = {}
    topic_times = {}
    one_msg_topics = []
    no_header_topics = {}
    for topic, msg, t in bag.read_messages():
        if hasattr(msg, 'header'):
            if topic not in topic_headers:
                topic_headers[topic] = [msg.header]
            else:
                topic_headers[topic].append(msg.header)
        else:
            if topic not in topic_headers:
                topic_times[topic] = [t.to_sec()]
            else:
                topic_times[topic].append(t.to_sec())

    for topic in topic_headers:
        if len(topic_headers[topic]) is 1:
            one_msg_topics.append(topic)
            continue
        print_highlight (topic + ": %d messages" % len(topic_headers[topic]))
        check_headers(topic_headers[topic])

    for topic in topic_times:
        if len(topic_times[topic]) is 1:
            one_msg_topics.append(topic)
            continue
        print_highlight (topic + ": %d messages without stamp" % len(topic_times[topic]))

    print_highlight ("There are other %d topics each having only one message" % len(one_msg_topics))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", '--verbose', dest='verbose', action='store_true')
    parser.set_defaults(verbose=False)
    args, left = parser.parse_known_args()
    if len(left) < 1:
        print ("Usage: " + sys.argv[0] + " [-v] filename [more ...]")
        exit()
    for bag in left:
        check_bag(rosbag.Bag(bag))
