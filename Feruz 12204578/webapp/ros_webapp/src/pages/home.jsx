import React, { Component } from 'react'
import Connection from '../componets/connection'
import Teleoperation from '../componets/teleoperation'
import MapView from '../componets/mapview'
import RobotStatistics from '../componets/robotstatistics'
import { Button } from 'react-bootstrap'
import { Row, Col } from 'react-bootstrap'
import myImage from '../images/map.png'
class Home extends Component {
	state = {}

	render() {
		return (
			<div>
				<Row className='nomargin  nopadding'>
					<Col className='nomargin nopadding box-border'>
						<Connection />
					</Col>
				</Row>
				<Row className='nomargin nopadding'>
					<Col
						className='nomargin nopadding box-border'
						style={{
							display: 'flex',
							justifyContent: 'space-around',
							alignItems: 'center',
						}}
					>
						<RobotStatistics className='nomargin box-border' />
						{/* <Button
							onClick={() => true}
							variant='outline-success'
							style={{ width: '180px', height: '60px' }}
						>
							Call the waiter
						</Button> */}
					</Col>
					<Col
						className='nomargin nopadding box-border'
						style={{
							display: 'flex',
							alignItems: 'center',
							justifyContent: 'center',
						}}
					>
						{/* <MapView /> */}
						<img src={myImage} style={{ width: '550px' }} />
					</Col>
				</Row>
				<Row>
					<Col>
						<Teleoperation />
					</Col>
				</Row>
			</div>
		)
	}
}

export default Home
