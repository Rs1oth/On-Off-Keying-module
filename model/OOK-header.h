
/* 
 *
 * Author: James Basuino   <jb392@njit.edu>
 *         
 *         
*/

#ifndef OOK_HEADER_H
#define OOK_HEADER_H

#include "ns3/header.h"

namespace ns3 {

class OOKHeader : public Header {
public:
	OOKHeader();
	virtual ~OOKHeader();
	static TypeId GetTypeId(void);
	virtual TypeId GetInstanceTypeId (void) const;
  	virtual void Print (std::ostream &os) const;
  	virtual void Serialize (Buffer::Iterator start) const;
  	virtual uint32_t Deserialize (Buffer::Iterator start);
  	virtual uint32_t GetSerializedSize (void) const;
	void SetProtocol(uint16_t protocol);
	uint16_t GetProtocol(void);
private:
	uint16_t m_protocol;
};

}

#endif
