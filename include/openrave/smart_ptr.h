#ifndef OPENRAVE_SMART_PTR_H
#define OPENRAVE_SMART_PTR_H

#define OPENRAVE_UNIQUE_PTR boost::unique_ptr
#define OPENRAVE_SHARED_PTR boost::shared_ptr
#define OPENRAVE_WEAK_PTR boost::weak_ptr
#define OPENRAVE_STATIC_POINTER_CAST boost::static_pointer_cast
#define OPENRAVE_ENABLE_SHARED_FROM_THIS boost::enable_shared_from_this
#define OPENRAVE_DYNAMIC_POINTER_CAST boost::dynamic_pointer_cast
#define OPENRAVE_CONST_POINTER_CAST boost::const_pointer_cast
#define OPENRAVE_MAKE_SHARED boost::make_shared
// std::function does not have "clear" method
#define OPENRAVE_FUNCTION boost::function

#endif // OPENRAVE_SMART_PTR_H