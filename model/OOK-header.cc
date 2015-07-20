
/* 
 *
 * Author: James Basuino  <jb392@njit.edu>
 *         
 *         
*/
#include <iostream>
#include "ns3/abort.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/header.h"
#include "OOK-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OOKHeader");
NS_OBJECT_ENSURE_REGISTERED(OOKHeader);

OOKHeader::OOKHeader(){}

OOKHeader::~OOKHeader(){}

TypeId OOKHeader::GetTypeId(void){
	static TypeId tid = TypeId("ns3::OOKHeader")
	.SetParent<Header>()
	.AddConstructor<OOKHeader>();
	return tid;
}

TypeId OOKHeader::GetInstanceTypeId(void) const
{
	return GetTypeId();
}

void OOKHeader::Print(std::ostream &os) const {
	std::string proto;
	switch(m_protocol){
		case 0x0021:
			proto = "IP (0x0021)";
			break;
		case 0x0057:
			proto = "IPv6 (0x0057)";
			break;
		default:
			NS_ASSERT_MSG(false, "OOK Protocol number not defined!");
	}
	os << "OOK Protocol: " << proto;
}

uint32_t OOKHeader::GetSerializedSize(void) const {
	return 2;
}

void OOKHeader::Serialize(Buffer::Iterator start) const {
	start.WriteHtonU16(m_protocol);
}

uint32_t OOKHeader::Deserialize(Buffer::Iterator start){
	m_protocol = start.ReadNtohU16();
	return GetSerializedSize();
}

void OOKHeader::SetProtocol(uint16_t protocol){
	m_protocol = protocol;
}

uint16_t OOKHeader::GetProtocol(void){
	return m_protocol;
}

}
