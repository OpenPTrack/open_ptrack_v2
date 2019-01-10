# This message is used to exchange time information between a ros node and 
# an android device that uses optar.

Header header

# The type of message. Either a request for the time or a reply to it
int32 type
int32 QUERY=0
int32 REPLY=1

# This id is needed to match a reply to its query. The server will use the query's id
# as the id for the reply. It should be unique among all the messages exchanged in the
# current ROS network. It's the client's responsibility to choose it.
string id

# The time on the server side. This will be filled up by the server when generating a reply
time serverTime

# The time on the client side at the time of the request
time clientRequestTime

